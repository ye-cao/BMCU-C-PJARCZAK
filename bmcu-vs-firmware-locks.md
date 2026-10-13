# BMCU, interoperability of Bambu Lab printers and restrictions introduced through firmware updates

## Contents

- [1. What BMCU is](#1-what-bmcu-is)
- [2. The firmware update problem](#2-the-firmware-update-problem)
- [3. New BUS certification and authorization frames](#3-new-bus-certification-and-authorization-frames)
- [4. Firmware versions mentioned](#4-firmware-versions-mentioned)
- [5. Scope of authorization control](#5-scope-of-authorization-control)
- [6. The main interoperability problem](#6-the-main-interoperability-problem)
- [7. BMCU behavior after the new restrictions](#7-bmcu-behavior-after-the-new-restrictions)
- [8. A1 firmware `01.08.00.00` and the slicer-side problem](#8-a1-firmware-01080000-and-the-slicer-side-problem)
- [9. Bambu Studio, AGPL and closed workflow components](#9-bambu-studio-agpl-and-closed-workflow-components)
- [10. X2D example and the ownership problem](#10-x2d-example-and-the-ownership-problem)
- [11. Beyond BMCU: the risk of firmware-controlled ecosystems](#11-beyond-bmcu-the-risk-of-firmware-controlled-ecosystems)
- [12. DJI analogy](#12-dji-analogy)
- [13. Legal context](#13-legal-context)

---

## 1. What BMCU is

BMCU is an **open-source, community multi-color system for Bambu Lab printers**. It is not an AMS clone. It is not AMS firmware running on different hardware. It is not a copy of an official Bambu Lab device. BMCU has its own firmware, its own hardware design and its own operating logic.

This distinction is very important, because in practice the whole discussion about BMCU is often incorrectly reduced to the phrase "AMS emulation". Technically, every device connected to the same bus has to speak to the printer in a way that the printer understands. This is how interoperability works. A USB keyboard speaks the USB HID protocol. A network device speaks the Ethernet/IP protocol. An accessory on a serial bus speaks the protocol of that bus. **The mere fact that a device communicates compatibly with the printer does not yet mean that it is a copy of the official product.**

BMCU was created by analyzing normal communication between a Bambu Lab printer and devices on the BUS. **This did not require extracting private keys, breaking encryption or stealing firmware.** The user has their own printer, their own device, their own bus and can observe communication that happens on their own hardware. On that basis, the community was able to build a separate multi-color device.

For such users, BMCU was a very sensible solution. It is smaller, cheaper, open, possible to build yourself and consistent with the DIY idea. The user can print parts, buy electronics, assemble the device and use it for their own needs. This is exactly the type of project that usually strengthens a hardware ecosystem, rather than weakening it. Many people bought cheap A1 or A1 mini printers precisely because the community around them was active, and projects like BMCU increased the usefulness of those printers.


---

## 2. The firmware update problem

> [!IMPORTANT]
> **The problem began when Bambu Lab started changing the operating rules of printers after purchase through printer firmware updates.**

BMCU worked without any problems on earlier firmware versions of first-generation printers. The printer was able to communicate with a device on the BUS and allowed the user to print. Not everything was perfect, but **there were no blocks from Bambu Lab.**

Then Bambu Lab started changing the operating rules of this hardware through firmware updates.

**On January 16, 2025, Bambu Lab officially announced a mechanism called the "Authorization Control System".** According to Bambu Lab, it was supposed to be a security mechanism. And sure - printer security is important. Nobody wants an unauthorized person to be able to remotely control the printer, move axes, change temperatures.

The problem is that in practice this mechanism is not limited only to a real threat from the internet. **It changes the operating rules of hardware and accessories after purchase.** It also affects devices connected locally to the printer, software from other producers, custom slicer builds and community projects.


---

## 3. New BUS certification and authorization frames

After this change, two new frames appeared in communication on the BUS:

- **0x040D - device certification frame.**
- **0x040E - device authorization frame.**

This is very important technically, because these frames were not previously present in the normal operating model of devices on the BUS.

After these frames were introduced, there is no longer only ordinary communication between devices. The printer can also ask **"is this device approved by Bambu Lab".**

Frame **`0x040D`** works as a certification stage. The printer sends a query to the device and expects a response in the form of a certificate. Such a certificate can be compared to a digital identity document of the device. The printer checks whether the device presents an identity that Bambu Lab recognizes as valid.

Frame **`0x040E`** works as an authorization stage. After certification, the printer can send a random control message to the device and expect the device to sign it with the correct key. If the response matches the certificate, the printer considers the device authorized. If it does not match - the device is rejected.

> [!IMPORTANT]
> **This is a fundamental change in the interoperability model.**

It is no longer only about whether the device talks to the printer correctly. **It is about whether the device is approved by the manufacturer.**


---

## 4. Firmware versions mentioned

Bambu Lab first announced this mechanism for the X1 series. Official materials mention X1 Series firmware **`01.08.03.00` or newer**.

Later the same direction was moved to other series.

For the P1 series, we are talking about firmware **`01.08.02.00`**.

For A1 and A1 mini, we are talking about firmware **`01.05.00.00`**.


---

## 5. Scope of authorization control

Bambu Lab itself indicated that authorization may apply to very important printer functions. This is not about one small operation. It includes, among other things:

- starting a print through LAN or cloud,
- axis movement,
- temperature control,
- fans,
- AMS settings,
- calibrations,
- firmware updates,
- access to remote preview.

So this is practical control over the most important functions of the printer.


---

## 6. The main interoperability problem

And here the fundamental problem appears.

> [!WARNING]
> **If a user bought a printer that allowed the use of devices on the BUS, and later a firmware update changes the rules and starts requiring manufacturer authorization, then the user loses part of the hardware interoperability after purchase.**

This is not a normal bug fix.

This is not just a new feature.

**This is a change to the operating rules of a device that has already been sold.**


---

## 7. BMCU behavior after the new restrictions

In the case of BMCU, this is very clear.

It is also worth paying attention to a very strange detail.

> [!NOTE]
> **Bambu Lab did not block BMCU immediately, even though after introducing device certification and authorization it could technically have done so already at the first attempt to use it.**

The printer allowed the first print with BMCU to start. That print could last an hour or even a month - for any length of time, as long as it was the same printing process.

This shows a very important thing: **the problem was not that BMCU did not work electrically or that it could not cooperate correctly with the printer.**

Since the printer could print with BMCU for an arbitrarily long first print, it is difficult to treat the later rejection of the device as an ordinary technical problem.

> [!WARNING]
> **It looks more like a deliberate firmware-side restriction of interoperability.**

And here the question appears: **why did Bambu Lab not block BMCU immediately from the first print?**

Was BMCU supposed to work like a demo version? Allow the user to start one print, see "wow, it works", and then cut the device off and push them towards buying the original, much more expensive AMS from Bambu Lab?

Subsequent prints could no longer start normally. The user had to reset the printer.

On A1 printers, the situation was specific for some time. Despite the introduced restrictions, BMCU could still be used in practice. The user saw errors or warnings related to AMS, but printing itself was possible. The printer did not disconnect BMCU from the bus in a way that completely prevented work after every print. It was inconvenient, but still usable.

So A1 users could continue to use BMCU in a way similar to how it worked before. There were HMS errors, there were warnings, but the physical printing function still worked.


---

## 8. A1 firmware `01.08.00.00` and the slicer-side problem

Later another update appeared.

**Firmware A1 `01.08.00.00` from April 14, 2026 introduced further problems.**

After this update, the printer started sending such a type of error to the slicer that it blocked the ability to print the current project. In practice, it looked like a critical error.

The behavior was partly random. Sometimes the error appeared, sometimes it did not. Sometimes the project or the whole program had to be closed, opened again, the model sliced again, and only then could the print be started.

This is exactly why the slicer modification makes sense.

The slicer does not repair the printer firmware, does not modify the printer, does not pass device certification, does not turn BMCU into an official AMS, **does not bypass the `0x040D` and `0x040E` frame mechanism.**

The slicer modification fixes the part of the problem that appears on the desktop workflow side.

If the printer sends an AMS/BMCU error, and the slicer treats that error as a state blocking the whole project, the user loses the ability to start the print normally. Even if physically the printer and BMCU could still perform the job.

The slicer modification is therefore about not allowing such an error to unnecessarily block the whole workflow. The slicer can ignore the blocking error state on the project side and allow the user to retry the print.

> [!IMPORTANT]
> **This is restoring practical usability, not bypassing device authorization, and this is a very important line.**

The goal is interoperability. The goal is to allow the user to use their own hardware in the same way that was available at the moment of buying the printer. **The goal is not to break security mechanisms.**


---

## 9. Bambu Studio, AGPL and closed workflow components

At this point, Bambu Studio also has to be mentioned.

Theoretically, modifying Bambu Studio should not be anything strange. **Bambu Studio is based on code from the PrusaSlicer and Slic3r family and is licensed under AGPL v3.** Bambu Lab did not write its slicer from scratch. They used an existing open-source project, with a long history and enormous community work behind it.

**The AGPL license gives users the right to analyze, modify and build their own versions of the program.**

That is the point of open-source.

The problem is that in practice Bambu Lab built a closed ecosystem around the slicer. If the user makes their own build or modifies Bambu Studio, important functions of the official workflow start to disappear. Normal cloud printing does not work, full preview in Bambu Handy does not work, and some functions depend on the closed bambu_networking component and a signed official build.

So again we see the same pattern.

**First the user receives functionality when buying the printer.**

**Then firmware or the ecosystem starts limiting it.**

**And when the user tries to use the rights resulting from open-source, it turns out that key elements are still controlled by closed, signed components from the manufacturer.**


---

## 10. X2D example and the ownership problem

This problem does not apply only to first-generation printers.

A very interesting example is also the second-generation X2D printer. This is important, because here we are not talking about an old printer that has been on the market for years. We are talking about a new model.

X2D at the moment of purchase may arrive to the user with firmware **`01.00.01.00`**.

On this firmware, BMCU works partially.

That is: the first print after starting the printer works. You can normally start a print and that print works. However, the next print already requires resetting the printer.

So we see exactly the same pattern: the first print works, because the printer still allows the device to operate, but after the print the firmware disconnects the device that does not pass authorization as a Bambu Lab device.

It is not ideal, but the user still has some functionality. If someone prints occasionally or sometimes wants to use BMCU for one print, they may decide that they can live with it.

> [!WARNING]
> **After updating to newer firmware, this partial functionality is completely blocked.** The printer requires device certification and authorization already before the first print.

And here the problem becomes even more serious.

If the user bought a printer that arrived with firmware **`01.00.01.00`**, saw or had confirmation that BMCU works on it at least partially, and then updated the firmware and that functionality disappeared, then from their perspective the update limited the hardware after purchase.

> [!WARNING]
> **Worse, after the update there is no normal path back to the previous version.** The official firmware history for the X2D printer does not even show the historical existence of the version with which the device physically arrived to the user.

This shows the ownership problem very well.

**Does the user buy a printer?**

**Or do they only buy access to a device whose operating rules the manufacturer can change through an update?**

Because if the printer worked with a certain accessory, and later a firmware update blocks that possibility and does not allow going back to the previous version, then the user loses practical hardware functionality after purchase.

This is not about requiring official support for BMCU from Bambu Lab.

This is not about Bambu Lab having to test BMCU, guarantee BMCU functionality or help BMCU users.

**This is about something simpler: not blocking functionality that previously worked on hardware belonging to the user.**

This is the core of the problem.

The manufacturer can say: "this is not our device, we do not support it, you use it at your own risk".

That is fair.

But lack of support is one thing, and a firmware update that after purchase restricts or blocks the use of a device that previously worked is another thing.

**This is the difference between lack of warranty and active restriction of interoperability.**


---

## 11. Beyond BMCU: the risk of firmware-controlled ecosystems

And that is exactly why BMCU is a good example of a bigger problem.

Today it is about open-source multi-color.

Tomorrow it may be about filament from another company.

Or about access to older functions that stopped being compatible with the manufacturer's policy.

If we accept a model in which the manufacturer can arbitrarily limit functionality after purchase under the banner of security, then the boundary starts disappearing.

> [!IMPORTANT]
> **Security is important, but it cannot be a universal excuse for closing an ecosystem.**

The real questions are specific:

- what exact security problem is being solved,
- why the solution is to restrict local interoperability,
- why the user loses a function that previously worked,
- why there is no "use at your own risk" mode,
- why there is no stable path for third-party hardware,
- why the manufacturer does not solve the real security problem without closing the whole ecosystem.


---

## 12. DJI analogy

It is also worth looking at the history of the team behind Bambu Lab.

Bambu Lab itself published a blog post titled ["The team behind Bambu Lab X1"](https://blog.bambulab.com/the-team-behind-bambu-lab-x1/#:~:text=DJI%20consumer%20drone%20department&text=System%20Engineering%20Department%20of%20DJI&text=DJI%20goggles%2C%20digital%20FPV%20systems&text=Before%20joining%20DJI%2C%20he%20worked%20at%20Marvell&text=DJI%20gimbal%20department&text=senior%20engineer%20of%20DJI&text=system%20design%20of%20DJI%20FPV%20remote%20controllers), in which it describes the founders and key people responsible for creating Bambu Lab printers. And there, a very strong connection with **DJI** appears directly.

Ye Tao worked on the DJI Mavic Pro and was the head of DJI's consumer drone division. Gao Xiufeng was connected with systems engineering at DJI. Liu Huaiyu worked on DJI Goggles, Digital FPV and FPV drones. Chen Zihan was connected with DJI gimbals.

This is not about saying that experience from DJI is itself something bad. Quite the opposite - technically, it is visible that Bambu Lab can make very polished hardware.

The problem lies elsewhere.

In DJI, a very similar way of thinking about the ecosystem can be seen: the hardware works well, but the manufacturer increasingly controls what the user can connect, what they can use and what firmware considers allowed.

A good example is DJI drone batteries.

In practice, it looked absurd. The user has a drone, has batteries, everything works. They perform a firmware update, because updates are supposed to fix bugs, improve stability and security. Then they go out into the field, want to fly normally, and only on site find out that the drone will not take off because the battery does not pass authorization.

**The hardware physically did not change, but the firmware changed the rules.**

And this is a very similar pattern to BMCU.

In the case of drone batteries, of course, one can say that the safety issue is more serious. If a battery fails in the air, the drone may fall. This is a real risk.

But even then, two things have to be distinguished.

One thing is a clear message:

"This is not our battery. We do not support it. We do not take responsibility for it. You use it at your own risk."

Another thing is a firmware update that after some time blocks or restricts the use of hardware that previously worked.

This is exactly the same difference we are talking about with BMCU.

Bambu Lab can say:

"We do not support BMCU. This is not our device. We do not guarantee operation. The user uses it at their own risk."

That would be a fair position.

But lack of official support is one thing, and introducing firmware updates that change the operating rules of the printer after purchase and restrict the use of devices that previously worked is another thing.

That is exactly why the analogy to DJI is important.

**Because it shows a certain model of thinking: the hardware is yours, but firmware and the ecosystem increasingly decide what you are allowed to do with that hardware.**

And this is a very dangerous direction.

**It is enough to say "security" and suddenly the manufacturer can try to justify almost any restriction.**


---

## 13. Legal context

In my opinion, such restrictions are not only a technical problem. They are also a legal problem.

In the European Union, the most important one is **Directive (EU) 2019/771** on the sale of goods. A 3D printer with firmware, updates, an application, an account, cloud and network functions is a good with digital elements. Such a product has to remain in conformity with the contract, and that conformity includes not only the fact that the device turns on, but also functionality, compatibility, interoperability and updates.

If before the update there was no mandatory certification and authorization of devices on the BUS, and after the update a mechanism appears that blocks independent devices, then in my opinion this is not an ordinary security update. **This is the use of firmware to restrict hardware interoperability after sale.**

The same applies to **Directive (EU) 2019/770**, if we look at the application, cloud, user account, remote access and the digital workflow around the printer. The producer should not, through a software or digital service change, take away the user's real access to functions that were previously a normal part of the product's operation.

There is also **Directive 2009/24/EC** on computer programs. EU law has long recognized interoperability as a legal goal. Independently created software and independent devices must be able to cooperate with an existing system, as long as this is not about copying the program or stealing code.

There is also the new **Directive (EU) 2024/825**, which is to apply from September 27, 2026. It goes even further and directly targets hiding information that a software update may negatively affect the operation of a product with digital elements or the use of digital content and services.

> [!IMPORTANT]
> **So the direction of EU law is clear: an update cannot be a hidden tool for making a product worse after purchase.**

In the USA, a similar direction can be seen in repair law. In the **"Nixing the Fix"** report, the **FTC** indicated **software locks**, **DRM**, technical protection measures and **firmware updates** as tools that can block consumers and independent repair shops. The **FTC** also announced enforcement against illegal repair restrictions, including under the **FTC Act**, antitrust law and the **Magnuson-Moss Warranty Act**.

The sense is very similar to the EU: **the manufacturer should not use software, firmware, authorization or signed components as a tool to take control over hardware after sale.**

In my opinion, such conduct violates the spirit of European rules on conformity of goods with digital elements, updates, functionality, compatibility and interoperability. In practice, the user buys a printer, and later the manufacturer uses firmware to restrict what that printer can do with locally connected hardware.

> [!CAUTION]
> **This is not a normal update - This is a change to the conditions of using the product after purchase.**