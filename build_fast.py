#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BMCU 固件快速批量编译工具
基于 DZRAB build_all_firmwares_fast.py 改造，适配本仓库 BMCU-C-PJARCZAK

加速原理：
  1. pio run -e moj -v 提取工具链路径与编译/链接命令行（带缓存）
  2. 预编译所有不受变体宏影响的 .o（只编一次）
  3. 按"模式组合"编译受宏影响的变体 .o 并缓存复用
  4. AMS_RETRACT_LEN 仅被 Motion_control 使用：用占位符 123.456f 编译基础固件，
     再在最终 .bin 中二进制修补该浮点为目标回抽长度
  5. 双开关(DM)自动回抽模式：每槽仅 1 个固件（92% 减少）
"""
import os
import re
import sys
import time
import struct
import shutil
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor

os.chdir(os.path.dirname(os.path.abspath(__file__)))
GLOBAL_START = time.perf_counter()

# ===== 配置 =====
FAST_MODE = True
OUT_DIR = "firmwares"
PIO_ENV = "fw"
PARALLEL_DIR = ".pio_parallel"
CACHE_DIR = os.path.join(PARALLEL_DIR, "obj_cache")
VERBOSE_CACHE = os.path.join(PARALLEL_DIR, "verbose_cache.txt")

TXT_MODE = "which_to_choose_mode.txt"
TXT_AUTOLOAD = "which_to_choose_autoload.txt"
TXT_RGB = "which_to_choose_filament_rgb.txt"
TXT_SLOTS = "which_to_choose_slots.txt"
OUT_GUIDE = "README.md"

SOLO_RETRACT = "0.095f"
RETRACTS = [
    "0.10", "0.15", "0.20", "0.25", "0.30", "0.35",
    "0.40", "0.45", "0.50", "0.55", "0.60", "0.65",
    "0.70", "0.75", "0.80", "0.85", "0.90", "0.95",
    "1.00", "1.05", "1.10", "1.15", "1.20", "1.25",
    "1.30", "1.35", "1.40", "1.45", "1.50", "1.55",
    "1.60", "1.65", "1.70", "1.75", "1.80", "1.85",
    "1.90", "1.95", "2.00",
]

VARIANT_MACROS = [
    "BAMBU_BUS_AMS_NUM", "AMS_RETRACT_LEN",
    "BMCU_DM_TWO_MICROSWITCH", "BMCU_ONLINE_LED_FILAMENT_RGB",
    "DBMCU_P1S", "BMCU_SOFT_LOAD",
    "BMCU_OLED",
]
RETRACT_MACRO = "AMS_RETRACT_LEN"
PLACEHOLDER_FLOAT = 123.456
PLACEHOLDER_BYTES = struct.pack("<f", PLACEHOLDER_FLOAT)

AUTO_RETRACT_CAP = "2.00"
auto_retract = int(os.environ.get("AUTO_RETRACT", "1"))

MODE_A1_DIR = "standard(A1)"
MODE_P1S_DIR = "high_force_load(P1S)"
MODE_SOFT_DIR = "soft_load(A1)"

MODES = [
    (MODE_A1_DIR, 0, 0),
    (MODE_P1S_DIR, 1, 0),
    (MODE_SOFT_DIR, 0, 1),
]

TPU_FIX = [os.environ.get(f"BMCU_TPU_FIX{i}", "GFU85").strip() or "GFU85" for i in range(4)]


def log(msg):
    print(msg, flush=True)


# ===== 工具链定位 =====
def find_pio_bin():
    pio = shutil.which("pio") or shutil.which("pio.exe")
    if pio:
        return pio
    home = os.path.expanduser("~")
    for cand in (os.path.join(home, ".platformio", "penv", "Scripts", "pio.exe"),
                 os.path.join(home, ".platformio", "penv", "bin", "pio")):
        if os.path.exists(cand):
            return cand
    log("ERROR: 找不到 pio，请先安装 PlatformIO 并加入 PATH")
    sys.exit(1)


PIO_BIN = find_pio_bin()

STARTUPINFO = None
if os.name == "nt":
    STARTUPINFO = subprocess.STARTUPINFO()
    STARTUPINFO.dwFlags |= subprocess.STARTF_USESHOWWINDOW


def run_cmd(args, check=True):
    r = subprocess.run(args, capture_output=True, startupinfo=STARTUPINFO)
    if check and r.returncode != 0:
        log(f"COMMAND FAILED: {' '.join(args[:5])} ...")
        log(r.stderr.decode(errors="replace")[:2000])
        return None
    return r


for f in (TXT_MODE, TXT_AUTOLOAD, TXT_RGB, TXT_SLOTS):
    if not os.path.exists(f):
        log(f"ERROR: 缺少必要描述文件 {f}")
        sys.exit(1)


# ====================================================================
# 第一步：提取工具链编译/链接参数（带缓存）
# ====================================================================
log("=" * 60)
log("  第一步：提取工具链编译/链接参数")
log("=" * 60)
os.makedirs(PARALLEL_DIR, exist_ok=True)


def _pio_ini_mtime():
    try:
        return os.path.getmtime("platformio.ini")
    except OSError:
        return 0


def _src_fingerprint():
    import hashlib
    h = hashlib.sha256()
    src_root = os.path.abspath("src")
    for root, _, files in os.walk(src_root):
        for fn in sorted(files):
            if fn.endswith((".c", ".cpp", ".h", ".cc", ".cxx", ".s", ".S")):
                p = os.path.join(root, fn)
                try:
                    h.update(os.path.relpath(p, src_root).replace("\\", "/").encode("utf-8"))
                    h.update(struct.pack("<Q", int(os.path.getmtime(p) * 1000)))
                except OSError:
                    pass
    return h.hexdigest()


SIG_PATH = os.path.join(PARALLEL_DIR, "verbose_cache.sig")

cache_valid = False
if (os.path.exists(VERBOSE_CACHE) and os.path.exists(SIG_PATH)
        and _pio_ini_mtime() < os.path.getmtime(VERBOSE_CACHE)
        and open(SIG_PATH, "r", encoding="utf-8").read().strip() == _src_fingerprint()):
    cache_valid = True

t_extract = time.perf_counter()
if cache_valid:
    log("  使用缓存的工具链参数（跳过 pio build）")
    with open(VERBOSE_CACHE, "r", encoding="utf-8") as f:
        verbose_output = f.read()
else:
    log("  运行 pio verbose build 提取参数（仅一次）...")
    run_cmd([PIO_BIN, "run", "-e", "moj", "-t", "clean"], check=False)
    res = run_cmd([PIO_BIN, "run", "-e", "moj", "-v"])
    if res is None:
        log("ERROR: 无法执行 PlatformIO 编译")
        sys.exit(1)
    verbose_output = res.stdout.decode(errors="replace") + res.stderr.decode(errors="replace")
    with open(VERBOSE_CACHE, "w", encoding="utf-8") as f:
        f.write(verbose_output)
    with open(SIG_PATH, "w", encoding="utf-8") as f:
        f.write(_src_fingerprint())
t_extract = time.perf_counter() - t_extract
log(f"  verbose 输出 {len(verbose_output)} 字符，提取耗时 {t_extract:.1f}s")

vo = verbose_output.replace("\\", "/")

cxx_line = cc_line = link_line = None
for line in vo.splitlines():
    line = line.strip()
    if not line:
        continue
    # Link line: contains firmware.elf, no -c flag
    if "firmware.elf" in line and link_line is None:
        link_line = line
        continue
    # Compile lines: must have -c and riscv toolchain
    if "-c" not in line:
        continue
    if "riscv" not in line or ("g++" not in line and "gcc" not in line):
        continue
    if "g++" in line and cxx_line is None:
        cxx_line = line
    elif "gcc" in line and "g++" not in line and cc_line is None:
        cc_line = line

if not all((cxx_line, cc_line, link_line)):
    log("ERROR: 无法从 verbose 输出提取编译/链接命令")
    sys.exit(1)

import hashlib
import os as _os

suffix = ".exe" if os.name == "nt" else ""

toolchain_bin = None
for part in cxx_line.split():
    m = re.search(r'(/.+?/bin/riscv.*?/)$', part.replace("\\", "/"))
    if m:
        toolchain_bin = m.group(1)
        break
if toolchain_bin is None:
    m = re.search(r'(/\S*?/bin/riscv)', cxx_line.replace("\\", "/"))
    if m:
        toolchain_bin = m.group(1)

# If verbose output doesn't have full path, find toolchain in .platformio
if toolchain_bin is None or not os.path.exists(os.path.join(toolchain_bin, "riscv-wch-elf-g++" + suffix)):
    home = os.path.expanduser("~")
    candidate = os.path.join(home, ".platformio", "packages", "toolchain-riscv", "bin")
    if os.path.isdir(candidate):
        toolchain_bin = candidate
        log(f"  工具链路径（从 .platformio 定位）: {toolchain_bin}")

if toolchain_bin is None or not os.path.exists(os.path.join(toolchain_bin, "riscv-wch-elf-g++" + suffix)):
    log("ERROR: 无法定位工具链")
    sys.exit(1)

CXX = os.path.join(toolchain_bin, "riscv-wch-elf-g++" + suffix)
CC = os.path.join(toolchain_bin, "riscv-wch-elf-gcc" + suffix)
AR = os.path.join(toolchain_bin, "riscv-wch-elf-gcc-ar" + suffix)
OBJCOPY = os.path.join(toolchain_bin, "riscv-wch-elf-objcopy" + suffix)
for tool in (CXX, CC, AR, OBJCOPY):
    if not os.path.exists(tool):
        log(f"ERROR: 工具链文件不存在: {tool}")
        sys.exit(1)


def parse_compile_flags(line, src_marker):
    tokens = line.split()
    try:
        c_idx = tokens.index("-c")
    except ValueError:
        c_idx = 0
    src_idx = len(tokens) - 1
    for i in range(len(tokens) - 1, c_idx, -1):
        if src_marker in tokens[i]:
            src_idx = i
            break
    raw = tokens[c_idx:src_idx]
    out = []
    skip = False
    for t in raw:
        if skip:
            skip = False
            continue
        if t == "-o":
            skip = True
            continue
        out.append(t)
    return out


def filter_variant_defines(flags):
    return [f for f in flags
            if not (f.startswith("-D") and any(m in f for m in VARIANT_MACROS))]


def fix_paths(flags):
    out = []
    for f in flags:
        if f.startswith("-I") and len(f) > 2:
            out.append("-I" + f[2:].replace("/", os.sep))
        elif os.sep == "\\" and "/" in f and (f.startswith("C:") or f.startswith("c:")):
            out.append(f.replace("/", os.sep))
        else:
            out.append(f)
    return out


raw_cxx = parse_compile_flags(cxx_line, "ADC_DMA")
raw_cc = parse_compile_flags(cc_line, "core_riscv")
CXX_BASE_FLAGS = fix_paths(filter_variant_defines(raw_cxx))
CC_BASE_FLAGS = fix_paths(filter_variant_defines(raw_cc))

# --- 提取链接参数 ---
link_tokens = link_line.split()
link_start = 0
for i, t in enumerate(link_tokens):
    if t.endswith("firmware.elf"):
        link_start = i + 1
        break

LINK_FLAGS = []
LINK_LD = None
obj_start = link_start
for i in range(link_start, len(link_tokens)):
    tok = link_tokens[i]
    if tok.endswith(".o"):
        obj_start = i
        break
    if tok == "-T":
        LINK_LD = link_tokens[i + 1].replace("/", os.sep) if i + 1 < len(link_tokens) else None
        continue
    if tok.endswith(".ld"):
        LINK_LD = tok.replace("/", os.sep)
        continue
    if tok.startswith("-"):
        if tok.startswith("-Wl,-Map"):
            continue
        LINK_FLAGS.append(tok)

if LINK_LD is None:
    LINK_LD = os.path.abspath(os.path.join(".pio", "build", "moj", "Link.ld"))
elif not os.path.isabs(LINK_LD):
    LINK_LD = os.path.abspath(LINK_LD)
if not os.path.exists(LINK_LD):
    cand = None
    for root, _, files in os.walk(os.path.join(".pio", "build")):
        if "Link.ld" in files:
            cand = os.path.join(root, "Link.ld")
            break
    LINK_LD = cand if cand else LINK_LD
if not os.path.exists(LINK_LD):
    log(f"ERROR: 链接脚本不存在: {LINK_LD}")
    sys.exit(1)

LINK_SUFFIX = []
in_suffix = False
for i in range(obj_start, len(link_tokens)):
    tok = link_tokens[i]
    if tok.endswith(".o"):
        continue
    if "-Wl,--start-group" in tok:
        in_suffix = True
    if in_suffix or tok.startswith("-L") or tok.startswith("-l") or tok.endswith(".a"):
        LINK_SUFFIX.append(tok.replace("/", os.sep) if (tok.endswith(".a") or tok.startswith("-L")) else tok)
    if "-Wl,--end-group" in tok:
        in_suffix = False

ORIG_FW_A = None
for t in link_tokens:
    if t.endswith(".a"):
        ORIG_FW_A = os.path.abspath(t.replace("/", os.sep))
        break
if ORIG_FW_A is None or not os.path.exists(ORIG_FW_A):
    log(f"ERROR: 找不到原始框架库: {ORIG_FW_A}")
    sys.exit(1)

# --- 收集所有编译命令 ---
compile_map = {}
framework_objs = {}
user_src_set = set()

for line in vo.splitlines():
    line = line.strip()
    if not line or "-c" not in line:
        continue
    if "riscv" not in line or ("g++" not in line and "gcc" not in line):
        continue
    tokens = line.split()
    obj_path = src_path = None
    for i, tok in enumerate(tokens):
        if tok == "-o" and i + 1 < len(tokens):
            obj_path = tokens[i + 1]
    for tok in reversed(tokens):
        if tok.endswith((".cpp", ".c", ".cc", ".cxx", ".s", ".S")):
            src_path = tok
            break
    if src_path is None:
        src_path = tokens[-1]
    if obj_path is None or src_path is None:
        continue
    obj_norm = obj_path.replace("\\", "/")
    src_norm = src_path.replace("\\", "/")
    abs_src = os.path.abspath(src_norm)
    src_root_abs = os.path.abspath("src")
    is_user = abs_src.lower() == src_root_abs.lower() or abs_src.lower().startswith(src_root_abs.lower() + os.sep)
    compiler = CXX if "g++" in line else CC
    flags = fix_paths(filter_variant_defines(parse_compile_flags(line, os.path.basename(src_norm))))
    compile_map[obj_norm] = (src_norm, compiler, flags)
    if is_user:
        user_src_set.add(src_norm)

link_obj_order = [t.replace("\\", "/") for t in link_tokens[obj_start:]
                  if t.endswith(".o")]

for obj_path, (src, compiler, flags) in compile_map.items():
    if src not in user_src_set:
        framework_objs[obj_path] = None


# ====================================================================
# 第二步：扫描源文件，按变体宏分类
# ====================================================================
log("=" * 60)
log("  第二步：扫描源文件，按变体宏分类")
log("=" * 60)

SRC_ROOT = os.path.abspath("src")


def scan_macros(src_file, seen=None):
    if seen is None:
        seen = set()
    found = set()
    path = os.path.abspath(src_file)
    if path in seen:
        return found
    seen.add(path)
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read()
    except OSError:
        return found
    for mac in VARIANT_MACROS:
        if re.search(r"(?<![\w])" + re.escape(mac) + r"(?![\w])", text):
            found.add(mac)
    for m in re.finditer(r'#\s*include\s*"([^"]+)"', text):
        inc = m.group(1)
        cand1 = os.path.join(os.path.dirname(path), inc)
        cand2 = os.path.join(SRC_ROOT, inc)
        for cand in (cand1, cand2):
            if os.path.exists(cand):
                found |= scan_macros(cand, seen)
                break
    return found


user_classification = {}
for src in user_src_set:
    macros = scan_macros(src)
    if RETRACT_MACRO in macros:
        user_classification[src] = "RETRACT"
    elif any(m in macros for m in VARIANT_MACROS if m != RETRACT_MACRO):
        user_classification[src] = "OTHER"
    else:
        user_classification[src] = "INVARIANT"

n_inv = sum(1 for v in user_classification.values() if v == "INVARIANT")
n_oth = sum(1 for v in user_classification.values() if v == "OTHER")
n_ret = sum(1 for v in user_classification.values() if v == "RETRACT")
log(f"  用户源文件: 不变 {n_inv} | 模式相关 {n_oth} | 含回抽长度 {n_ret}")
log(f"  框架/SDK 源文件: {len(framework_objs)}")
log(f"  链接 .o 数量: {len(link_obj_order)}")


# ====================================================================
# 第三步：收集任务 & 创建输出目录
# ====================================================================
log("=" * 60)
log("  第三步：收集任务 & 创建输出目录")
log("=" * 60)

tasks = []
for mode_dir, p1s, soft_load in MODES:
    for oled in (1, 0):
        oled_base = os.path.join(OUT_DIR, "noled") if oled == 0 else OUT_DIR
        for dm in (1, 0):
            dm_dir = "AUTOLOAD" if dm == 1 else "NO_AUTOLOAD"
            for rgb in (1, 0):
                rgb_dir = "FILAMENT_RGB_ON" if rgb == 1 else "FILAMENT_RGB_OFF"
                base = os.path.join(oled_base, mode_dir, dm_dir, rgb_dir) if oled == 0 else os.path.join(OUT_DIR, mode_dir, dm_dir, rgb_dir)
                if dm == 1 and auto_retract:
                    tasks.append((os.path.join(base, "SOLO", f"solo_{AUTO_RETRACT_CAP}f_auto.bin"),
                                  0, f"{AUTO_RETRACT_CAP}f", dm, rgb, p1s, soft_load, oled))
                    for slot, ams_num in (("A", 0), ("B", 1), ("C", 2), ("D", 3)):
                        tasks.append((os.path.join(base, f"AMS_{slot}", f"ams_{slot.lower()}_{AUTO_RETRACT_CAP}f_auto.bin"),
                                      ams_num, f"{AUTO_RETRACT_CAP}f", dm, rgb, p1s, soft_load, oled))
                else:
                    tasks.append((os.path.join(base, "SOLO", f"solo_{SOLO_RETRACT}.bin"),
                                  0, SOLO_RETRACT, dm, rgb, p1s, soft_load, oled))
                    for slot, ams_num in (("A", 0), ("B", 1), ("C", 2), ("D", 3)):
                        for r in RETRACTS:
                            tasks.append((os.path.join(base, f"AMS_{slot}", f"ams_{slot.lower()}_{r}f.bin"),
                                          ams_num, f"{r}f", dm, rgb, p1s, soft_load, oled))

total_tasks = len(tasks)
log(f"  总计 {total_tasks} 个固件编译任务")

if os.path.exists(OUT_DIR):
    shutil.rmtree(OUT_DIR, ignore_errors=True)
os.makedirs(OUT_DIR, exist_ok=True)
shutil.copy(TXT_MODE, os.path.join(OUT_DIR, OUT_GUIDE))
for mode_dir, p1s, soft_load in MODES:
    for oled_base in (OUT_DIR, os.path.join(OUT_DIR, "noled")):
        mode_base = os.path.join(oled_base, mode_dir)
        os.makedirs(mode_base, exist_ok=True)
        shutil.copy(TXT_AUTOLOAD, os.path.join(mode_base, OUT_GUIDE))
        for dm in (1, 0):
            dm_dir = "AUTOLOAD" if dm == 1 else "NO_AUTOLOAD"
            dm_base = os.path.join(mode_base, dm_dir)
            os.makedirs(dm_base, exist_ok=True)
            shutil.copy(TXT_RGB, os.path.join(dm_base, OUT_GUIDE))
            for rgb in (1, 0):
                rgb_dir = "FILAMENT_RGB_ON" if rgb == 1 else "FILAMENT_RGB_OFF"
                b = os.path.join(dm_base, rgb_dir)
                os.makedirs(b, exist_ok=True)
                shutil.copy(TXT_SLOTS, os.path.join(b, OUT_GUIDE))
                for slot in ("SOLO", "AMS_A", "AMS_B", "AMS_C", "AMS_D"):
                    os.makedirs(os.path.join(b, slot), exist_ok=True)


# ====================================================================
# 第四步：并行预编译所有 .o
# ====================================================================
log("=" * 60)
log("  第四步：并行预编译所有 .o 文件")
log("=" * 60)

if os.path.exists(CACHE_DIR):
    shutil.rmtree(CACHE_DIR, ignore_errors=True)
os.makedirs(CACHE_DIR)
COMMON_OBJ_DIR = os.path.join(CACHE_DIR, "common")
os.makedirs(COMMON_OBJ_DIR, exist_ok=True)


def compile_one(compiler, flags, src, obj_path):
    obj_abs = os.path.abspath(obj_path)
    src_abs = os.path.abspath(src) if not os.path.isabs(src) else src
    os.makedirs(os.path.dirname(obj_abs), exist_ok=True)
    cmd = [compiler, "-o", obj_abs] + flags + [src_abs]
    r = subprocess.run(cmd, capture_output=True, startupinfo=STARTUPINFO)
    if r.returncode != 0:
        log(f"\n  COMPILE FAILED: {os.path.basename(src)}")
        log(r.stderr.decode(errors="replace")[:1000])
        return False
    return True


def variant_defines(dm, rgb, p1s, soft_load, ams_num, retract="0.095f", oled=1):
    defines = [
        f"-DBMCU_DM_TWO_MICROSWITCH={dm}",
        f"-DBMCU_ONLINE_LED_FILAMENT_RGB={rgb}",
        f"-DBMCU_P1S={p1s}",
        f"-DBMCU_SOFT_LOAD={soft_load}",
        f"-DBAMBU_BUS_AMS_NUM={ams_num}",
        f"-DAMS_RETRACT_LEN={retract}",
        f"-DBMCU_OLED={oled}",
    ]
    if dm == 1 and not auto_retract:
        defines.append("-DBMCU_DM_AUTO_RETRACT=0")
    fix_defs = [f"-DBMCU_TPU_FIX{i}={TPU_FIX[i]}" for i in range(4)]
    defines += fix_defs
    return defines


def vkey_of(dm, rgb, p1s, soft_load, ams_num, oled=1):
    return f"dm{dm}_rgb{rgb}_p1s{p1s}_sl{soft_load}_ams{ams_num}_oled{oled}"


compile_tasks = []

invariant_user_map = {}
for obj_path, (src, compiler, flags) in compile_map.items():
    if src in user_src_set and user_classification.get(src) == "INVARIANT":
        obj = os.path.join(COMMON_OBJ_DIR, "user", os.path.basename(obj_path))
        os.makedirs(os.path.dirname(os.path.abspath(obj)), exist_ok=True)
        compile_tasks.append((compiler, flags, src, obj, f"inv:{os.path.basename(obj_path)}"))
        invariant_user_map[obj_path] = obj

for obj_path in list(framework_objs.keys()):
    framework_objs[obj_path] = os.path.abspath(obj_path.replace("\\", "/"))

mode_combos = set()
for (_, ams_num, _, dm, rgb, p1s, soft_load, oled) in tasks:
    mode_combos.add((dm, rgb, p1s, soft_load, ams_num, oled))

other_user_map = {}
retract_user_map = {}

for combo in sorted(mode_combos):
    dm, rgb, p1s, soft_load, ams_num, oled = combo
    vkey = vkey_of(dm, rgb, p1s, soft_load, ams_num, oled)
    vdir = os.path.join(CACHE_DIR, "variant", vkey)
    os.makedirs(vdir, exist_ok=True)
    defs = variant_defines(dm, rgb, p1s, soft_load, ams_num, oled=oled)
    defs_ph = variant_defines(dm, rgb, p1s, soft_load, ams_num, retract=f"{PLACEHOLDER_FLOAT}f", oled=oled)
    for obj_path, (src, compiler, flags) in compile_map.items():
        if src not in user_src_set:
            continue
        grp = user_classification.get(src)
        if grp == "OTHER":
            obj = os.path.join(vdir, "other_" + os.path.basename(obj_path))
            compile_tasks.append((compiler, flags + defs, src, obj, f"oth:{vkey}/{os.path.basename(obj_path)}"))
            other_user_map[(obj_path, vkey)] = obj
        elif grp == "RETRACT":
            obj = os.path.join(vdir, "ret_" + os.path.basename(obj_path))
            compile_tasks.append((compiler, flags + defs_ph, src, obj, f"ret:{vkey}/{os.path.basename(obj_path)}"))
            retract_user_map[(obj_path, vkey)] = obj

log(f"  共 {len(compile_tasks)} 个编译任务")

compile_jobs = os.cpu_count() or 4
pre_done = 0
pre_lock = threading.Lock()
total_pre = len(compile_tasks)
t_pre = time.perf_counter()


def _do_compile(task):
    global pre_done
    compiler, flags, src, obj, name = task
    ok = compile_one(compiler, flags, src, obj)
    with pre_lock:
        pre_done += 1
        cur = pre_done
    sys.stdout.write(f"\r  [预编译] {cur}/{total_pre} | {cur*100//total_pre}% | {time.perf_counter()-t_pre:.1f}s")
    sys.stdout.flush()
    return name if not ok else None


with ThreadPoolExecutor(max_workers=compile_jobs) as ex:
    results = list(ex.map(_do_compile, compile_tasks))
print("")
failed_pre = [r for r in results if r]
if failed_pre:
    log(f"ERROR: {len(failed_pre)} 个源文件预编译失败")
    sys.exit(1)
t_pre = time.perf_counter() - t_pre
log(f"  预编译完成 {total_pre} 个 .o，耗时 {t_pre:.1f}s")

missing_fw = [o for o in framework_objs.values() if not os.path.exists(o)]
if missing_fw:
    log(f"ERROR: {len(missing_fw)} 个框架 .o 未生成")
    sys.exit(1)


# ====================================================================
# 第五步：链接基础固件 + 二进制修补回抽长度
# ====================================================================
log("=" * 60)
log("  第五步：链接基础固件 + 修补回抽长度")
log("=" * 60)

link_sem = threading.Semaphore(min(4, compile_jobs))


def build_link_objs(vkey, elf_path=None):
    objs = []
    for obj_path in link_obj_order:
        src, _, _ = compile_map[obj_path]
        if src in user_src_set:
            grp = user_classification.get(src)
            if grp == "INVARIANT":
                objs.append(invariant_user_map[obj_path])
            elif grp == "OTHER":
                objs.append(other_user_map[(obj_path, vkey)])
            elif grp == "RETRACT":
                objs.append(retract_user_map[(obj_path, vkey)])
            else:
                log(f"ERROR: 未分类的链接对象 {obj_path}")
                sys.exit(1)
        else:
            objs.append(framework_objs[obj_path])
    if elf_path is None:
        tmp_dir = os.path.join(CACHE_DIR, "tmp_base", vkey)
        os.makedirs(tmp_dir, exist_ok=True)
        elf_path = os.path.join(tmp_dir, "firmware.elf")
    else:
        os.makedirs(os.path.dirname(os.path.abspath(elf_path)), exist_ok=True)
    link_cmd = [CXX, "-o", os.path.abspath(elf_path), "-T", LINK_LD] + LINK_FLAGS \
        + [os.path.abspath(o) for o in objs]
    for tok in LINK_SUFFIX:
        if tok.endswith(".a"):
            link_cmd.append(ORIG_FW_A)
        else:
            link_cmd.append(tok)
    link_env = os.environ.copy()
    link_env["TMP"] = os.path.abspath(os.path.dirname(elf_path))
    link_env["TEMP"] = os.path.abspath(os.path.dirname(elf_path))
    link_env["TMPDIR"] = os.path.abspath(os.path.dirname(elf_path))
    with link_sem:
        r = subprocess.run(link_cmd, capture_output=True, startupinfo=STARTUPINFO, env=link_env)
    if r.returncode != 0:
        err = r.stderr.decode(errors='replace')[:1500]
        log(f"\n  LINK FAILED {vkey}:\n{err}")
        raise RuntimeError(f"LINK FAILED {vkey}: {err}")
    return elf_path


base_bins = {}
base_done = [0]
prog_lock = threading.Lock()
t_link = time.perf_counter()


def run_base_link(combo):
    dm, rgb, p1s, soft_load, ams_num, oled = combo
    vkey = vkey_of(dm, rgb, p1s, soft_load, ams_num, oled)
    elf_path = build_link_objs(vkey)
    bin_path = os.path.join(os.path.dirname(elf_path), "firmware.bin")
    with link_sem:
        subprocess.run([OBJCOPY, "-O", "binary", elf_path, bin_path],
                       capture_output=True, startupinfo=STARTUPINFO, check=True)
    with open(bin_path, "rb") as f:
        data = f.read()
    with prog_lock:
        base_done[0] += 1
        cur = base_done[0]
    sys.stdout.write(f"\r  [基础链接] {cur}/{len(mode_combos)} | {cur*100//len(mode_combos)}% | {time.perf_counter()-t_link:.1f}s")
    sys.stdout.flush()
    return vkey, data


link_errors = []


def _base_worker(combo):
    last = None
    for attempt in range(2):
        try:
            return run_base_link(combo)
        except Exception as e:
            last = e
            log(f"  [warn] 基础链接重试 {combo}: {e}")
    link_errors.append(str(last))
    return None


with ThreadPoolExecutor(max_workers=compile_jobs) as ex:
    for res in ex.map(_base_worker, sorted(mode_combos)):
        if res:
            base_bins[res[0]] = res[1]

print("")
if link_errors:
    log(f"ERROR: 基础链接失败 {len(link_errors)} 个")
    sys.exit(1)

t_link = time.perf_counter() - t_link
log(f"  基础链接完成 {len(base_bins)} 个模式组合，耗时 {t_link:.1f}s")


# ====================================================================
# 第六步：二进制修补 + 复制固件
# ====================================================================
log("=" * 60)
log("  第六步：二进制修补 + 复制固件")
log("=" * 60)

failed_builds = []
done_patch = 0
t_patch = time.perf_counter()
total_patch = len(tasks)


def patch_and_copy(task):
    global done_patch
    out_path, ams_num, retract_str, dm, rgb, p1s, soft_load, oled = task
    vkey = vkey_of(dm, rgb, p1s, soft_load, ams_num, oled)
    base_data = base_bins.get(vkey)
    if base_data is None:
        return f"NO BASE for {vkey}"

    data = bytearray(base_data)
    retract_val = float(retract_str.rstrip("f"))
    target_bytes = struct.pack("<f", retract_val)

    pos = data.find(PLACEHOLDER_BYTES)
    if pos == -1:
        return f"PLACEHOLDER not found in {vkey}"
    data[pos:pos+4] = target_bytes

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(data)

    with prog_lock:
        done_patch += 1
        cur = done_patch
    sys.stdout.write(f"\r  [修补] {cur}/{total_patch} | {cur*100//total_patch}% | {time.perf_counter()-t_patch:.1f}s")
    sys.stdout.flush()
    return None


with ThreadPoolExecutor(max_workers=compile_jobs) as ex:
    results = list(ex.map(patch_and_copy, tasks))

print("")
failed_builds = [r for r in results if r]
if failed_builds:
    log(f"ERROR: {len(failed_builds)} 个固件修补失败:")
    for f in failed_builds[:10]:
        log(f"  {f}")

t_patch = time.perf_counter() - t_patch


# ====================================================================
# 清理 & 汇总
# ====================================================================
if os.path.exists(CACHE_DIR):
    shutil.rmtree(CACHE_DIR, ignore_errors=True)

total_time = time.perf_counter() - GLOBAL_START
tm, ts = divmod(int(total_time), 60)
te, tse = divmod(int(t_extract), 60)
tp, tsp = divmod(int(t_pre), 60)
tl, tls = divmod(int(t_link), 60)
log("=" * 60)
log("  编译完成")
log("=" * 60)
log(f"  固件总数 : {total_tasks}")
log(f"  成功     : {total_tasks - len(failed_builds)}")
log(f"  失败     : {len(failed_builds)}")
log(f"  输出目录 : {OUT_DIR}/")
log("-" * 60)
log(f"  提取参数 : {te}分{tse}秒")
log(f"  预编译.o : {tp}分{tsp}秒 ({total_pre} 个)")
log(f"  链接基础 : {tl}分{tls}秒 ({len(mode_combos)} 个)")
log(f"  修补/复制: {t_patch:.1f}秒")
log(f"  总耗时   : {tm}分{ts}秒")
log("=" * 60)
