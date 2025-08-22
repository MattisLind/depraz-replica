# Depraz mouse replica

Based on the work by John Floren this project is aiming at creating a modern replica of the Depraz mouse. A mouse that outputs quadrature signals exactly as the original.

The physical 3D model will be the same as the Bellwether mouse by John but the circuit board is replaced. The new circuitboard is going to use an STM32F103 micro controller. Since this processor is using 3.3V IO no level shifters are necessary. 

# Old Bellwether project

This repo should contain everything you need to make a [Bellwether mouse](https://jfloren.net/bellwether.html).

Note that I've made several of these myself, but it's possible that I've forgotten a step or component somewhere.

I'm willing to answer questions to the best of my ability. I'll also consider assembling some portions of the mouse and mailing it out; send me an email if you want this.

The 3d models and the circuit are CC BY-NC-SA licensed. I believe this means that "group buys" (where 1 person coordinates ordering a bunch of PCBs and parts) are ok under this model.

## 3D-printed case

Inside the case/ subdirectory you'll find two FreeCAD files, "base" (the bottom part of the mouse) and "body" (the curved top which fits in your hand).

These should be printed in PLA. I've had good luck with my Prusa Mini. I strongly suggest using a raft when printing the body, and *not* using a raft for the base.

Check out the releases for pre-built STL files.

## PCB

The board was designed using KiCAD. If you use OSH Park, you should be able to just upload board/depraz_board.kicad_pcb directly on their site. For other PCB services, you may need to open KiCAD and export Gerber files etc.

## Circuit & BOM

You can check out the schematic in schematic.pdf

bom.csv is a circuit bill of materials. Most stuff should be available from parts companies in the US, but for the PMW3389 sensor you'll probably need to go to AliExpress (e.g. https://www.aliexpress.us/item/2251832705021246.html)

You'll also need 3 momentary pushbutton switches such as https://www.digikey.com/en/products/detail/5501M1BLKX/EG1701-ND/101654

Although you don't have to, I recommend using a DIP socket for the microcontroller in case you toast the microcontroller.

## Assembly

### Circuit assembly

In general it's all pretty straightforward but I strongly recommend installing the SMD level shifters first, all the rest of the components next, and then do the mouse sensor *last*.

### Doing the switches

I recommend gluing down the switches first. Gently pry off the button from each switch and set it aside. Then put a small dab of Gorilla Glue in the slot on the base, press the switch onto it, and make sure it's aligned straight. Use a clamp to hold the switch in place while the glue dries (I used something like https://www.amazon.com/Professional-Plastic-Backdrop-Photography-Improvement/dp/B08RXW32H4/). Once the glue is dry, make sure the switch is firmly seated and use an exacto knife to trim away any glue which oozed out, then press the button back onto the switch face.

Solder two wires onto each switch, on the normally-open pins. The wires should have Dupont pin connectors on the ends; I just took some jumper wires I had lying around (with a connector on each end) and cut them in half. See http://jfloren.net/content/bellwether/base2.jpg

### Physical assembly

You'll need the following parts to assemble the case:

* 8x M2x0.4mm heatserts: https://www.mcmaster.com/94459A120/
* 4x M2x0.4mm flathead screws (for the bottom): https://www.mcmaster.com/91294A003/
* 4x M2x0.4mm button head screws (to hold the board down): https://www.mcmaster.com/92095A451/

Additional stuff you'll want:

* A USB cable. Should be USB-A on one end, and a female pin header connector at the other. You can start with https://www.digikey.com/en/products/detail/assmann-wsw-components/A-USB20AM-OE-200BK28/10408422 and use https://www.amazon.com/Taiss-Ratcheting-Connector-Crimping-Terminal/dp/B0B11RLGDZ to crimp on a 4-pin header to the other end. You may be able to find these pre-made, e.g. https://www.amazon.com/dp/B07ZPT2NMT
* Friction tape (https://www.amazon.com/gp/product/B0000CBIAT) to wrap around the USB cable where it leaves the mouse. This appears to be what my original Depraz mouse used.
* Teflon tape to make skates for the bottom. I used https://www.amazon.com/gp/product/B01LZ44P6M and punched out pieces using https://www.amazon.com/gp/product/B00006IBK9

Use your soldering iron to install the heatserts flush in the corresponding holes of the case.

Position the lens on the bottom of the mouse sensor, then set the PCB on the 3d printed base. Use the button-head screws to hold down the PCB; don't overtighten!

Plug in the switches. Note that the headers on the board are labeled R, M, and L for right, middle, and left.

Plug the USB cable into the 4-pin USB header. Lay it across the board to where the cable hole in the case will go; mark this point and put a layer or two of friction tape on the cable there.

Gently slide the top of the case over the switches; there's a "right way" to do it, which takes a bit of gentle experimentation to find. Once you get the two pieces together, use flathead screws to hold them together.

Punch out four pieces of Teflon tape and stick them to the bottom for skates.

### Flashing the firmware

The firmware organization sucks, but I am not good enough at PIC programming to properly clean it up. PRs welcome.

Plug your PIC programmer into the PROGRAM header on the board.

Open MPLAB X IDE and load firmware/depraz/apps/usb/device/hid_mouse/firmware/low_pin_count_usb_development_kit_pic16f1459.x

I found that it worked when I set up the PICkit3 to provide power at 3.5V. Your other option is to plug in the USB cable before programming to power the device externally.

### Adding heft

You'll probably find that the mouse by itself is way too light. I hot-glued a bunch of lead fishing weights to the inside and it was much more pleasant.
