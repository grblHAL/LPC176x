## LPC176x my_machine, Re-ARM boards

### Pin assignmets for [RAMPS 1.6](https://reprap.org/wiki/RAMPS_1.6) board.

__Note 1:__ Limit pins are not interrupt capable so hard limits cannot be enabled.  
__Note 2:__ Limit pins are wired to external pullup resistors, MCU pulldown cannot be used.  
__Note 3:__ Control pins \(Reset, Feed hold, Cycle start) do not support MCU pullup/pulldown resistors.

#### Work in progress, suggestions for improvements welcome!

Uncomment `#define BOARD_RAMPS16` in [my_machine.h](../main/my_machine.h) to use this mapping.

``` plain

                                                             [  ]
                                                             [  ] GND
                                          RESET [  ]    R    [13] B4.28
                                            3V3 [  ]    E    [12] P2.12
                                            3V3 [  ]    -    [11] P1.20
                                            GND [  ]    A    [10] P2.5 - Spindle PWM
                                            GND [  ]    R    [09] P2.4
                                            VIN [  ]    M    [08] P2.7
                                                        
                                  X step - P2.1 [A0]         [07] -
                                  X dir - P0.11 [A1]    L    [06] P1.21
                               Y enable - P0.19 [A2]    P    [05] P1.19 - Spindle dir
                                Reset** - P0.27 [A3]    C    [04] P1.18 - Spindle enable
                            Feed hold** - P0.28 [A4]    1    [03] P1.24 - X limit min
                             Cycle start - P2.6 [A5]    7    [02] P1.25 - X limit max
                                  Y step - P2.2 [A6]    6    [01] P0.2 - TXD0
                                  Y dir - P0.20 [A7]    8    [00] P0.3 - RXD0

                               Z enable - P0.21 [A8]         [14] P1.26 - Y limit min
                                Coolant - P0.26 [A9]         [15] P1.27 - Y limit max
                                             - [A10]         [16] P0.16
                                             - [A11]         [17] P0.18
                                             - [A12]         [18] P1.29 - Z limit min
                                         P0.23 [A13]         [19] P1.28 - Z limit max
                                         P0.24 [A14]         [20] P0.0
                                         P0.25 [A15]         [21] P0.1

P0.10 - X enable ------------------------------------+     +-------------------------------------------- B step - P2.8 (E2)
    - ------------------------------------------+    |     |    +---------------------------------------- B dir - P2.13 (E2)
    - -------------------------------------+    |    |     |    |    +------------------------------------------- -
    - --------------------------------+    |    |    |     |    |    |    +--------------------------- B enable - P4.29 (E2)
 P2.3 - Z step ------------------+    |    |    |    |     |    |    |    |    +------------------------- A dir - P0.5 (E1)
P0.22 - Z dir --------------+    |    |    |    |    |     |    |    |    |    |    +------------------- A step - P2.0 (E1)
P0.17 ------- ---------+    |    |    |    |    |    |     |    |    |    |    |    |    +------------ A enable - P0.4 (E1)
P0.15 ------------+    |    |    |    |    |    |    |     |    |    |    |    |    |    |    +------------------ -
           [GND] [52] [50] [48] [46] [44] [42] [40] [38] [36] [34] [32] [30] [28] [26] [24] [22] [3V3]
           [GND] [53] [51] [49] [47] [45] [43] [42] [39] [37] [35] [33] [31] [29] [27] [25] [23] [3V3]
P1.23 - Aux 0* ---+    |    |    |    |    |    |    |     |    |    |    |    |    |    |    +------------------ P0.15
P0.18 - Aux 1* --------+    |    |    |    |    |    |     |    |    |    |    |    |    +----------------------- -
P1.31 - Aux 2* -------------+    |    |    |    |    |     |    |    |    |    |    +---------------------------- -
    - ---------------------------+    |    |    |    |     |    |    |    |    +--------------------------------- -
    - --------------------------------+    |    |    |     |    |    |    +-------------------------------------- P3.26
    - -------------------------------------+    |    |     |    |    +------------------------------------------- P3.25
P1.22 ------------------------------------------+    |     |    +------------------------------------------------ P2.11
    - -----------------------------------------------+     + ---------------------------------------------------- P1.30

```

\* Aux output pins are optional and must be enabled in the board map file.
\*\* Pull-up or pull-down cannot be enabled for these pins.

---
2023-08-24
