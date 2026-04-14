Image /page/0/Picture/0 description: The image shows the official logo and name of Glasgow College, UESTC (University of Electronic Science and Technology of China). On the left side, there is a circular emblem/seal in dark navy blue. The circular seal contains the text "Glasgow College, UESTC" along the bottom arc and Chinese characters "电子科技大学格拉斯哥学院" along the top arc. Inside the circle is the University of Glasgow crest/shield logo with the text "University of Glasgow" and the year "2013" below it, indicating the year the college was established. To the right of the seal, the name is displayed in large Chinese calligraphy-style characters: "电子科技大学" (University of Electronic Science and Technology of China) on the top line and "格拉斯哥学院" (Glasgow College) on the second line, followed by the English text "Glasgow College, UESTC" beneath. The overall color scheme is dark navy blue on a white background.

# UESTCHN 3018 Team Design Project and Skills

**Lecture 1b**

**Path-following robots with automotive radar – An Overview**

## Introduction

On this course you will be required to **design**, **build** and **demonstrate** a robot car that performs certain tasks.

The tasks will be demonstrated using **1 patio** which will have a total of **3 tasks** that are further divided into **10 subtasks**.

The maximum cost of the project is **2500 RMB**.

- You are free to design and implement a car of your choice that accomplishes the tasks in this project. All parts should be listed in the bill of materials, which should be an appendix in the team final report.
- Excellent projects will provide full justification for the choice of components used.

## Example car components

Image /page/3/Picture/1 description: This image is a diagram showing the key components of a robotics project, arranged in a circular layout. The five components are: 1) A \*\*4-Wheel Chassis\*\* (top-left) — a black rectangular platform with four yellow-hubbed wheels with black rubber tires and an antenna; 2) \*\*Motors\*\* (top-right) — three silver DC gear motors with yellow gearboxes and protruding shafts; 3) \*\*Sensors\*\* (bottom-left) — a green PCB (printed circuit board) with a glowing green circular sensor element in the center; 4) \*\*Microcontroller\*\* (bottom-center) — a dark-colored development board with gold pin headers along both sides and visible electronic components; 5) \*\*Battery Pack\*\* (bottom-right) — a green cylindrical rechargeable battery with a red and black wire connector. The components are arranged around a thin gray circular arc, suggesting they all connect or integrate together to form a complete robot. At the bottom of the image, the text reads "UESTCHN 3018" on the left and "Glasgow College Hainan, UESTC" in the center, indicating this is an academic or course-related presentation slide.

## Patio - top view

Image /page/4/Figure/1 description: This image depicts a symmetrical robotics competition course map with a mirrored layout. The course has two green 'Start' squares located at the bottom-left and bottom-right corners, and a central red 'Finish' rectangle in the middle of the map.

The track is composed of black lines forming a complex path that includes circular elements (resembling roundabouts or traffic circles stacked vertically), rectangular obstacles/zones, curved S-shaped paths, and Y-shaped fork/intersection symbols. The left half of the course is a mirror image of the right half.

Numbered checkpoint markers are placed along the course, color-coded by task:
- Orange markers (Task 1: Line following): labeled 1.1 (near top of the outer vertical paths), 1.2 (near the bottom Start areas), 1.3 (mid-level on the inner vertical paths), 1.4 (near the top of the inner paths leading to the center), and 1.5 (above the Finish area in the center).
- Purple markers (Task 2: Wireless communications): labeled 2.1 (near each Start area at the bottom) and 2.2 (on either side of the central Finish rectangle).
- Yellow/gold markers (Task 3: Automotive radar): labeled 3 (on each side, positioned near rectangular zones in the upper-middle portion of the course).

Each side of the course features a tall vertical loop at the outer edge, a column of stacked circles (4 circles), rectangular blocks along the inner path, and curving connectors leading to the center. The legend at the bottom reads: 'Task 1: Line following.' (in dark red/maroon), 'Task 2: Wireless communications.' (in purple), and 'Task 3: Automotive radar.' (in yellow/gold).

## Competition overview

This year, two teams will be racing, starting from both ends of the track. The tracks are symmetrical. The racing part is optional and does not contribute to the final marks. The cars will be only assessed based on technical performance during the designed tasks. So, the racing is for the "fun" part. The best 6 teams will be selected to participate in the finals on the demonstration day, and the best three teams will be given an award

certificate.

The 6 finalist teams will be selected based on their demonstration scores and the total time it took their cars to finish all three tasks.

## Patio - top view - with dimensions

Image /page/6/Figure/1 description: This is a detailed technical schematic/blueprint of a robotics obstacle course or competition track, viewed from above, with a green 'Start' box in the bottom-left corner and a red 'Finish' box on the right side. The course features numerous obstacles and path segments with precise measurements in centimeters.

\*\*Start (bottom-left):\*\* A green square labeled 'Start' (50cm × 50cm), with a red line extending to the right.

\*\*Course sections and obstacles (traced from Start to Finish):\*\*

1. \*\*Serpentine/wavy path section (labeled 1.2):\*\* From Start, the path enters a section with rectangular blocks arranged vertically. The blocks are 50cm wide with 60cm vertical spacing. The corridor is 70cm wide, with 50cm gaps between blocks. The wavy black path weaves between these blocks.

2. \*\*Straight vertical corridor with U-turn (labeled 1.1):\*\* A tall narrow corridor (70cm + 70cm wide, 200cm tall) on the upper left with a rounded U-turn at the top. Two scissor/cross-shaped obstacles are positioned within at the 70cm width mark.

3. \*\*Circular obstacles section (labeled 1.3):\*\* A vertical column of four circles (approximately 30cm diameter each) spaced 45cm apart, enclosed within a rectangular boundary (50cm wide, 98cm tall).

4. \*\*Large rectangular obstacle (labeled 1.4):\*\* A large box measuring 120cm wide × 70cm, positioned in the upper-right area, with 73.5cm vertical spacing above and 165cm horizontal span.

5. \*\*Loop/turn section:\*\* A curved path with 30cm radius turns leading to a lower square obstacle (50cm × 50cm) on the right side, with 50cm clearances.

6. \*\*Finish (right side, labeled 1.5):\*\* A red rectangle labeled 'Finish' (100cm × 50cm), connected by a 50cm horizontal path.

\*\*Overall dimensions (top header):\*\* 70cm + 70cm + 50cm + 165cm + 100cm horizontally. Vertically: 200cm (upper) + 250cm (lower) + 50cm (start area). A 30.5cm measurement is noted at the bottom center.

All paths are shown as thick black lines, and five numbered waypoints (1.1 through 1.5) are marked at key positions along the course.

## Task 1 – Line following

Task 1. From the start point (green box), your car needs to follow the black line. It also needs to navigate through the bends and boxes. Ideally, you would want to follow the shortest route to the finish box. The line will be 3 cm thick, and the green box will be 50 x 50 cm in size. This task has 5 subtasks with a combined total of 50% of the total demonstration marks.

What kind of components are needed to achieve that? A camera, IR or???

「hink!

Image /page/7/Picture/3 description: The image is a schematic diagram or course layout showing a path from a green 'Start' box at the bottom-left to a red 'Finish' box at the right-center. The route is divided into five numbered sections marked with orange/peach circular labels (1.1, 1.2, 1.3, 1.4, and 1.5). 

Section 1.1 (top-left): Features a tall, narrow U-shaped hairpin loop at the top, with two vertical parallel lines going upward and curving around. There are two crossing/scissors-like symbols on the vertical lines partway down.

Section 1.2 (bottom-center): Contains a serpentine/slalom path weaving through approximately four offset rectangles/squares arranged vertically, with curved connecting lines between them.

Section 1.3 (center): Shows a vertical column of five circles of varying sizes stacked within a rectangular frame, connected by lines.

Section 1.4 (upper-right): Features a large horizontal rectangle connected to the circle section above and leading downward through a U-shaped connector and a small square element.

Section 1.5 (right): Located near the red 'Finish' box, connected via a line from section 1.4's path.

The overall path flows from Start upward through 1.1, down through 1.2, up through 1.3, across to 1.4, and finally to 1.5 and the Finish. The diagram resembles an obstacle course, climbing route, or agility course layout.

## Task 1 – Line following

**Task 1.** The car should be on top of the line at all times. If the car fully leaves the line, the subtask must be restarted from the previous marker. e.g. if it completely leaves the line between 1.2 and 1.3, it should start again from 1.2.

What kind of components are needed to achieve that? A camera, IR or???

Image /page/8/Picture/3 description: The image depicts a course or flowchart-style diagram illustrating a path from 'Start' to 'Finish,' divided into five numbered sections (1.1 through 1.5). 

- \*\*Start\*\* is marked with a green square box at the bottom-left corner.
- \*\*Finish\*\* is marked with a red rectangular box on the right side.

\*\*Section 1.1\*\* (top left): A tall, narrow vertical U-shaped loop path that goes upward and curves back down, with two scissor-like or cross-shaped gate symbols along the vertical lines.

\*\*Section 1.2\*\* (bottom center): A serpentine/wavy path weaving through a series of staggered square obstacles arranged in a descending zigzag pattern.

\*\*Section 1.3\*\* (center): A vertical rectangular frame containing a column of approximately 5-6 stacked circles of varying sizes, resembling a slalom or vertical obstacle course.

\*\*Section 1.4\*\* (upper right): A large horizontal rectangle connected to the path, appearing as a wide open area or checkpoint.

\*\*Section 1.5\*\* (right, near Finish): Features a vertical line with a loop/U-turn at the bottom and a small square obstacle, leading to the red 'Finish' box.

The sections are connected by black lines indicating the route, and each section is labeled with an orange circle containing the section number (1.1, 1.2, 1.3, 1.4, 1.5). The overall layout suggests a sequential obstacle course or agility-type challenge diagram.

## Task 2 – Wireless communications

At markers 2.1 and 2.2, there will be an arch which is 50 cm high. The arch will have a wireless receiver pointing downwards. When your car passes underneath the arch, it must transmit the current time, team number and name and the time duration since the start of the race in minutes:seconds. This task will carry 20% of the total demonstration marks.

Image /page/9/Picture/2 description: The image shows a diagram featuring three rectangles arranged vertically in a staggered pattern, connected by lines. The top rectangle is positioned slightly left of center, the middle rectangle is offset to the right (overlapping with the top rectangle), and the bottom rectangle is positioned below and slightly left, also overlapping with the middle rectangle. On the left side, a vertical line descends from the top and transitions into a wavy or sinusoidal curve that runs downward alongside the rectangles. At the bottom of the diagram, there is a purple circle with the number '2.1' written in white inside it. Below the circle, a horizontal black arrow points to the right. The overall layout suggests a flowchart or process diagram demonstration, with the partial text 'demonstration' visible at the top of the image (cropped).

Image /page/9/Picture/3 description: The image shows a simple flowchart or process diagram. On the left side, there is a vertical path that starts from a square/rectangle shape at the bottom, goes upward through a straight vertical line, then forms a U-shaped loop (hairpin turn) at the top. From the top of this loop, a horizontal arrow points to the right toward a purple circle labeled '2.2'. This purple circle connects to a large red rectangle labeled 'Finish' in white text. The overall layout suggests a process flow that begins at the bottom square, travels upward through a looping path, passes through step 2.2, and terminates at the Finish block.

## Task 2 – Wireless communications

The transceiver module that you need to use is the (Ebyte EWM22A-400900BWL22S) which is based on the LoRa (Long Range) protocol.

The LoRa protocol is a low-power, long-range wireless modulation technology designed for IoT, operating in unlicensed sub-gigahertz radio bands. The centre frequency is 868 MHz and the frequency band is 850 – 930 MHz.

The receiver will be provided and integrated within the arch. You are only required to buy and program the transmitter.

## Task 3 – Automotive radar

In task 3, there will be an obstacle (a metal plate which is 50 cm wide and 70 cm high) placed randomly at one side of the box, either right or left. The car should scan the box ahead and determine where the obstacle is placed before entering the box. It should use the free side of the box to pass through to the next stage. This task will carry 30% of the total demonstration marks.

Image /page/11/Picture/2 description: A technical schematic or plumbing/piping diagram. On the left side, there is a vertical rectangular enclosure containing four circles stacked vertically, representing pipes or cylinders viewed from the side. From the top of this left structure, a horizontal pipe extends to the right and then turns downward. A red arrow indicates the flow direction: horizontally to the right along the top, then vertically downward on the right side. A black double-headed vertical arrow labeled '73 cm' marks the distance between the top horizontal pipe and the top edge of a rectangular box/component positioned on the right side of the diagram. This rectangular box has a yellow circle with the number '3' on its right side. Below this box, a curved pipe loops downward and connects to a fitting at the bottom right. The left vertical structure also has a pipe extending downward from its bottom. A partial letter 'e' is visible on the far left edge of the image.

What direction/angle should the radar face? Is one radar enough or two are required?

## Task 3 – Automotive radar

For this task, a **24 GHz radar chip** must be used. Higher frequencies are also allowed but nothing below 24 GHz can be used. This year, TDPS will will be kindly supported by CYCPLUS, a leading company in developing FMCW radar modules. (datasheet and documentation available on Moodle). We also recommend this tested commercial chip (HMMD-mmWave sensor HLK-LD2410S). You are free to use other 24 GHz radar chips.

Image /page/12/Figure/2 description: A close-up photograph of a green printed circuit board (PCB) enclosed in a black rectangular housing/enclosure. The PCB is labeled 'L7\_MM\_V1.0 20230615' indicating it is version 1.0 dated June 15, 2023. The board features several prominent millimeter-wave antenna patches arranged in a 2x2 array pattern in the center-left area, each consisting of gold/copper rectangular patch elements with feed lines on a dark substrate. There is a central IC chip (likely a radar or mmWave sensor) surrounded by small surface-mount components labeled C2G, R1U, U2, C8, C15, C10, C9, among others. Connector pads labeled P1 and P2 are visible at the top-right and bottom-center respectively. Along the right edge of the PCB, there are several small white SMD LEDs or connectors arranged vertically. Additional component labels visible include B18, B20, references to inductors (L), and other passive components along the left edge. A cable exits from the top-right corner of the enclosure. The board appears to be a millimeter-wave radar module, possibly used for sensing or communication applications.

Image /page/12/Picture/4 description: A small blue and gold PCB (printed circuit board) labeled "HMMD-mmWave-Sensor" with the designation "D03" visible on the board. The board features a central black QFN-packaged microchip/IC connected via gold traces to two large gold rectangular antenna patches at the top of the board, which serve as millimeter-wave radar antennas. Several small SMD (surface mount device) components are visible, labeled R14, C15, R15, R16, R13, C14, C3, C8, and others. Along the bottom edge, there is a row of pin headers (connector J2) with labeled pins: 3V3, GND, TX, RX, and OT2. The board has a compact, roughly square form factor with rounded corners, and the PCB substrate is dark blue with gold/copper traces and pads. The board number "012" is also printed near the bottom right edge.

What direction/angle should the radar face? Is one radar enough or two are required? CYCPLUS radar chip HLK-LD2410S

## Approved batteries

Batteries should only be ordered from the two approved brands: **Tattu R-line and Ovonic.** For safety reasons, the maximum permitted battery **capacity is 5000 mAh.** The estimated capacity required for the project is approximately 2400 mAh, so the 5000 mAh limit provides sufficient headroom. Ideally, you should select a battery with enough capacity to run the car for multiple sessions, with each run lasting around 10 minutes. A battery that provides approximately 30 minutes to 1 hour of total runtime should therefore be sufficient.

What battery capacity you need that can keep your car running for 1 hour?

Image /page/13/Picture/3 description: The image shows two LiPo (Lithium Polymer) batteries commonly used for FPV drones, placed side by side for comparison. On the left is a Tattu R-Line Version 4.0 battery, which is 1550mAh, 130C discharge rate, 4-cell (4S) configuration at 14.8V with 22.94Wh capacity, model number TAA15504S13X6. It has a yellow and black label and features an XT60 connector (yellow). On the right is an Ovonic FPV High Discharge LiPo battery, also 1550mAh but with a 100C discharge rate, in a 6-cell (6S1P) configuration at 22.2V with 34.41Wh capacity. It has a black and green label with a helicopter/drone graphic and also features an XT60 connector. Both batteries have balance lead connectors (white plugs) and red/black power wires visible.

## LCD - TFT – OLED screens

Using a pre-programmed route to complete the linefollowing task is **strictly prohibited**. The vehicle must detect and follow the line using a camera or sensor. A **4-inch onboard (LCD, TFT or OLED) screen** must be integrated to display the camera or sensor output used for line tracking, as well as the radar signature.

At the start of the run, the screen should display the camera or sensor vision used for line detection. When the car reaches **Checkpoint 1.4**, the display must switch to the radar sensor and show either a waterfall or spectrogram signature of the radar not raw data. After the vehicle passes the obstacle box, the display should switch back to the camera or sensor vision.

Image /page/14/Picture/3 description: A small TFT LCD display module mounted on a blue PCB (printed circuit board). The screen is displaying a vibrant image of a deep space nebula scene with bright blue and white stars scattered across a dark blue cosmic background. The PCB has mounting holes in each corner and a row of pin headers along the left side for connecting to a microcontroller or development board. There is also a small connector or component visible on the right side of the board. This appears to be a touchscreen LCD shield commonly used with Arduino or similar embedded platforms, likely 3.5 to 4 inches in size.

**Example screen**

Image /page/14/Picture/5 description: The image shows an Arduino-based radar/sonar project assembled on a white breadboard. The setup includes an Arduino board (appears to be an Arduino Uno or Nano) in the center, connected to a small TFT LCD display on the left side that shows a green radar/sonar sweep animation with concentric circles and a scanning line. On the right side, an HC-SR04 ultrasonic distance sensor (recognizable by its two cylindrical transducers) is mounted on a small servo motor, allowing it to rotate and scan the surroundings. Multiple colorful jumper wires (rainbow colored - red, orange, yellow, green, blue, purple) connect the components together. A USB cable with a standard USB-A connector is visible in the foreground, used for programming the Arduino and providing power. There is also a dark circular overlay in the upper-left corner of the image. The background is dark gray/charcoal colored.

**Example of screen mounted on car**

## Penalties

During the final demo, all components will be checked for compliance, any team not meeting the requirements above will be **penalised by up to 25%** of demo marks for each breach of requirements.

Use of approved batteries (Tattu R-line or Ovonic), sensors/camera for line tracking, LoRa transmitter for wireless communication, 24 GHz radar for object detection, and a screen to display sensor data should be adhered to.

- 1. The project demonstration for all the teams will be held on **Friday 12 June 2026**.
  - Each team will train and program their car to complete all tasks on the Patio.
  - All teams should be prepared to demonstrate their car at 8:30 am and at other times when their team is called later in the day.
  - Teams should have their batteries fully charged before 8:30 am. Use of power packs to supplement or replace the battery will not be allowed.

- 2. Each team will have **one** opportunity to complete the tasks on their assigned Patio. Additional opportunities to complete the tasks on a particular patio may be allowed if time permits. However, there is **no guarantee** that this will be allowed.
  - **Each team will be allotted 10 minutes per** run on assigned Patio for the team's car to complete all the tasks.
  - Every effort will be made to announce when a team should begin a run on their assigned patio, but it is the team's responsibility to check their scheduled demonstration time and be ready at the starting point of the first task. A team that fails to begin the run within the 5-minute window will be given a score of 0 for the first (and possibly only) run on that course.

- 3. The car must run using a program that has been previously downloaded to a microcontroller on board the car. Instructions cannot be transmitted in real-time to the car.
- 4. You can use any board within the budget for this project.
- 5. A functional robot must be demonstrated by week 8 during the presentations, it should be able to move forward and does not have to have any integrated sensors. Just basic movement forward.
- 6. Week 13, we will carry a mock demonstration to trial the robots. This will be arranged by the GTAs.

- 5. There are **three tasks (10 subtasks)** to be completed on the Patio.
  - You will receive 10 full marks for any subtask only if you complete it in the first run. You will be penalized by two marks for every external interference, touch or restart.
  - To receive full marks for the all subtasks, the transition from one subtask to another should be programmed (without any external interference). You will be penalized for any repositioning or restart (minus two marks for each external interference, touch or restart) for all subsequent subtasks if it does not transit automatically from one to another.

- 6. Electrical systems and all connections to circuit components and subsystems must be rugged and reliable.
  - Wires should be soldered onto PCBs, V-board, or punchboard or screwed into terminal blocks on PCBs, V-board or punchboard. Breadboard circuits are not allowed.
  - A **fuse and ON-OFF switch** should be placed between the battery and the rest of the car. The fuse should be sized appropriately so that it will not be damaged during normal operation of the car, but will create an open circuit should more current than expected be drawn from the battery.
  - All risk mitigation elements should be considered during the design process.
- 7. Each team is expected to design a motor driver circuit and the PCB on which this circuit is constructed.

## Lab Books

• You are required to keep an updated record of your project progress. Your lab books **will be assessed in week 8.**

• "*Researchers use a lab notebook to document their hypotheses, experiments and initial analysis or interpretation of these experiments. The notebook serves as an organizational tool, a memory aid, and can also have a role in protecting any intellectual property that comes from the research* " – **Wikipedia**.

## Lab Books

• Paper based Laboratory Notebooks

• Electronic Laboratory Notebooks

Image /page/22/Picture/3 description: The image shows two laboratory notebooks. In the foreground, a closed notebook with a black hardcover is displayed, with the title "LABORATORY NOTEBOOK" printed in white capital letters centered on the front cover. Behind it, a second laboratory notebook with a dark blue or navy cover is open, revealing its interior pages. The open pages feature a light green grid/graph paper layout with printed column headers and fields at the top and bottom margins for recording information such as dates, titles, and signatures — typical of scientific laboratory record-keeping notebooks. The notebooks are photographed against a plain white background in a product-style presentation.

Image /page/22/Figure/4 description: This image shows a chemistry reaction planning software interface (likely an electronic lab notebook or reaction scheme editor) displaying "Step 1" of a multi-step synthesis. The top section shows a reaction scheme with four chemical structures labeled A through D, depicting a reaction involving methyl 2-(2-(5-chloropyrimidin-4-yl)oxyphenyl) compound (A, MW 278.7), MeI (B, MW 141, CAS 74-88-4), Ethyl Formate (C, MW 74.0784, 98 mass%), and a product D (methyl (E)-2-(2-(5-chloropyrimidin-4-yl)oxy... MW 320.7, 100 mass%). The reaction arrows show A + B + C yielding D.

On the left side, there is a procedural checklist with steps including: adding to a 3-neck RBF, equipped with N2 inlet, adding reagents via syringe, heating to reflux, cooling, warming, stirring overnight, analyzing via TLC, workup procedures (EtOAc, DCM), purification via flash chromatography, and analytical steps (HPLC, NMR, GC/MS).

Below the reaction scheme is a table listing reagents with columns for CAS#, Density, Purity/Conc, and % Yield, showing 4 rows of materials.

An "Import Materials" dialog box is open in the foreground, with search options set to "DiscoveryGate ACD" source and "Name or CAS# List" type. The search results panel displays a table with columns for Selected, Structure, Density, Purity/Co., MW, MF, CAS#, and Diluent. Several entries are shown with MW 107.125, MF C4H14u N, CAS 4111-54-0, with densities around 0.812-0.80 g/mL. A section labeled "Name: THF" shows entries with density 0.89 g/mL, 99.5 mass%, MW 72.1062, MF C4H8O, CAS 109-99-9. Checkboxes indicate selected materials, with Select All, Uncheck All, OK, and Cancel buttons at the bottom.

## Project Planning – Gantt Charts

- GANTT charts display the tasks in a project as a box or line showing the calendar duration of the task on the horizontal axis (the horizontal length of the task box is proportional to the task duration)
- Tasks are normally arranged in date order on the vertical axis
- The time relation of all tasks to each other (for example, tasks carried out simultaneously) is therefore clearly apparent in a GANTT chart

## Project Planning – Gantt Charts

- The project status can be easily determined at intermediate dates in the project
- Progress of individual tasks can be shown by filling in the task boxes.
- Dependencies between tasks can be indicated by lines linking tasks.

## **Gantt Charts - Examples**

Image /page/25/Figure/1 description: This image is a Gantt chart displayed on a light pink background, showing a project schedule spanning six weeks from March 3 to April 11. The timeline header shows weekly date ranges (Mar 3-7, Mar 10-14, Mar 17-21, Mar 24-28, Mar 31-Apr 04, Apr 7-11) with individual weekdays (M, T, W, T, F) marked in light blue cells beneath each week.

The chart is organized into three main task groups, each with a dark navy/black header label:

\*\*Task 1\*\* (3 activities):
- Activity 1 (assigned to TP): A blue bar spanning roughly Mar 3–7, followed by a magenta/pink bar from approximately Mar 10–12.
- Activity 2 (assigned to JO): A magenta bar from approximately Mar 10–14, then a purple/violet bar from approximately Mar 17–21.
- Activity 3 (assigned to PY): A magenta bar from approximately Mar 12–14, a purple bar from approximately Mar 17–21, and a hot pink/red bar from approximately Mar 24–27.

\*\*Task 2\*\* (3 activities):
- Activity 1 (assigned to TL): A purple bar from approximately Mar 17–21, then a small pink bar around Mar 24–25.
- Activity 2 (assigned to CS): A small purple bar around Mar 19–21, a pink bar from approximately Mar 24–28, and a light pink bar from approximately Mar 31–Apr 2.
- Activity 3 (assigned to PY): A pink bar from approximately Mar 26–28, a light pink/peach bar from approximately Mar 31–Apr 4, and a light blue bar from approximately Apr 7–9.

\*\*Task 3\*\* (4 activities):
- Activity 1 (assigned to CS): A purple bar from approximately Mar 17–21.
- Activity 2 (assigned to TL): A hot pink bar from approximately Mar 21–28.
- Activity 3 (assigned to PY): A light pink/peach bar from approximately Mar 31–Apr 4.
- Activity 4 (assigned to TL): A light pink/peach bar from approximately Mar 31–Apr 4, followed by a light blue bar from approximately Apr 7–11.

Each activity has a purple circle on the far right containing the initials of the assigned person (TP, JO, PY, TL, CS). The color coding of the bars appears to represent different phases or progress stages, transitioning from blue to magenta to purple to pink to light pink to light blue across the timeline.

## Gantt Charts - Examples

Image /page/26/Figure/1 description: This image is an annotated Gantt Chart diagram used as an educational or explanatory illustration of Gantt chart components. The chart has a light pink background and is titled "Gantt Chart" in bold black text at the top.

The chart contains a table on the left with columns: TASKS, START, END, and WBS. Six rows of placeholder data are listed: TASKS 01 through TASKS 03 (with DATE 01–03 and WBS 01–03), and STEP 04 through STEP 06 (with DATE 04–06 and WBS 04–06).

The timeline header spans from JAN to JUL across the top, with each month displayed in a different color: JAN (teal/dark cyan), FEB (blue/periwinkle), MAR (purple/magenta), APR (pink/rose), MAY (light purple), JUN (light blue), and JUL (light brown/tan).

The chart area shows horizontal task bars in black and light pink representing task durations. Tasks 01 and 02 span roughly from JAN to early MAR. Task 03 spans from JAN to APR with a milestone marker (black star) near APR. Steps 04–06 are staggered later, with Step 04 in MAR-APR, Step 05 from MAR to MAY, and Step 06 from APR to JUL. Progress within tasks is shown by black filled portions within outlined bars.

A vertical dashed pink line labeled "TODAY" is positioned at approximately early March.

Annotated callouts with arrows point to key elements: "Start and End Dates" points to the START/END columns, "Project timeline" points to the month headers, "Task list" points to the TASKS column, "Dependencies" points to connector lines between tasks, "Today line" points to the dashed vertical line, "Progress" points to the filled portion of a task bar, "Milestone" points to the star symbol, and "Task bars" points to the horizontal bars in the chart area. Dependencies are shown as angled connector lines linking the end of one task to the start of another.

## Process flow diagram- Examples

| Symbol               | Name         | Function                                                                          |
|----------------------|--------------|-----------------------------------------------------------------------------------|
| Image: Rectangle     | Process      | A rectangle represents a process.                                                 |
| Image: Oval          | Start/end    | An oval represents a start or end point.                                          |
| Image: Diamond       | Decision     | A diamond indicates a decision.                                                   |
| Image: Parallelogram | Input/Output | A parallelogram represents input or output.                                       |
| Image: Arrow         | Arrows       | A line is a connector that shows relationships between the representative shapes. |

## Process flow diagram- Examples

Image /page/28/Figure/1 description: This image is a flowchart depicting the operational process of a beacon-following robot with obstacle avoidance capabilities. The flow begins at the top with a rounded 'START' box, which leads down to 'INITIATE THE ROBOT' (gray parallelogram-style box), then to 'LIVE STREAM THE VIDEO' (gray box). The video stream feeds into a 'CENTRAL SYSTEM' block on the right side, illustrated with an image of a laptop computer on a light blue background.

From 'LIVE STREAM THE VIDEO,' the flow proceeds to 'MOVE FORWARD & FOLLOW THE BEACON' (gray box), which leads to an orange diamond-shaped decision block labeled 'OBSTACLE DETECTED?'. If 'No,' the flow loops back to 'MOVE FORWARD & FOLLOW THE BEACON.'

If an obstacle is detected, the flow proceeds downward to a blue rectangular box labeled 'FUZZY-BASED OPTIMAL PATH.' This box receives inputs from three blue sensor boxes on the left: 'L - ULTRASONIC SENSOR' (left), 'F - ULTRASONIC SENSOR' (front), and 'R - ULTRASONIC SENSOR' (right). After a 'DELAY,' the output goes to 'TURN TOWARDS THE BEACON DIRECTION' (gray box), which feeds back to both the 'CENTRAL SYSTEM' and loops back into the main flow. The Central System also connects back to the movement and beacon-following steps. The flowchart appears to be cut off at the bottom, suggesting additional decision points below.