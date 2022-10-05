[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

# Awesome Canadian Robotics

A curated list of Canadian robotics open-source software, compagnies and researchers.
The maintenance of this list is aligned with the proposition of the [Canadian Robotics Council](https://www.roboticscouncil.ca) to better share information about robotics initiative in Canada.
This is not a comprehensive list of everything happening in Canada related to robotics.
We added abitrary criterias to be listed here with the intention to be transparent.

## Contents

- [Open-Source Software](#open-source-software)
- [Datasets](#datasets)
- [Companies](#companies)
  - [Robot manufacturers](#robot-manufacturers)
  - [Natural Resources](#natural-resources)
  - [Warehouse Robotics](#warehouse-robotics)
- [Organisations and Divisions](#organisations-and-divisions)
- [Laboratories](#laboratories)
- [Related Awesome Lists](#related-awesome-lists)

## Open-Source Software

You need at least 15 stars on your repository to be listed here.
The list is sorted by number of stars.

- [![GitHub Repo stars](https://img.shields.io/github/stars/ethz-asl/libpointmatcher?style=social)](https://github.com/ethz-asl/libpointmatcher/stargazers) [libpointmatcher](https://github.com/ethz-asl/libpointmatcher): 
  - An Iterative Closest Point (ICP) library for 2D and 3D mapping in Robotics. [Norlab](https://norlab.ulaval.ca) is maintaining and using the library for their research on autonomous navigation in harsh environments.
- [![GitHub Repo stars](https://img.shields.io/github/stars/nickcharron/lidar_snow_removal?style=social)](https://github.com/nickcharron/lidar_snow_removal/stargazers) [lidar_snow_removal](https://github.com/nickcharron/lidar_snow_removal): 
  - This repo is a set of nodes for ROS to filter point clouds with the goal of removing snow in Lidar data. [TrailLAb](https://www.trailab.utias.utoronto.ca/) was maintaining this package for their publications on the [Canadian Adverse Weather Dataset](http://cadcd.uwaterloo.ca/).
- [![GitHub Repo stars](https://img.shields.io/github/stars/utiasASRL/vtr3?style=social)](https://github.com/utiasASRL/vtr3/stargazers) [VT&R3](https://github.com/utiasASRL/vtr3): 
  - VT&R3 is a C++ implementation of the Teach and Repeat navigation framework. It enables a robot to be taught a network of traversable paths and then closely repeat any part of the network.  [utiasASRL](https://utiasasrl.github.io/) is maintaining and using the package for their research on vision-based localization algorithms in outdoor environments.
- [![GitHub Repo stars](https://img.shields.io/github/stars/SFU-MARS/optimized_dp?style=social)](https://github.com/SFU-MARS/optimized_dp/stargazers) [optimized_dp](https://github.com/SFU-MARS/optimized_dp):  
  - Optimizing Dynamic Programming-Based Algorithms Resources. [SFU-MARS](https://sfumars.com/research/) is maintaining this library for their research on principled robot decision making ombining traditional analytical methods in robotics and modern data-driven techniques.
- [![GitHub Repo stars](https://img.shields.io/github/stars/MRASL/mrasl_mav_traj?style=social)](https://github.com/MRASL/mrasl_mav_traj/stargazers) [mrasl_mav_traj](https://github.com/MRASL/mrasl_mav_traj): 
  - Trajectory utilities for Micro UAVs (MAVs). Maintained by [MRASL](http://www.polymtl.ca/robotique-mobile/en).
- [![GitHub Repo stars](https://img.shields.io/github/stars/SherbyRobotics/pyro?style=social)](https://github.com/SherbyRobotics/pyro/stargazers) [pyro](https://github.com/SherbyRobotics/pyro): 
  - An object-based toolbox for robot dynamic simulation, analysis, control and planning. [Createk](https://www.createk.co/) and Prof. Alexandre Girard is maintaining this library for their research on robot design, control and planning.
- [![GitHub Repo stars](https://img.shields.io/github/stars/norlab-ulaval/wiln?style=social)](https://github.com/norlab-ulaval/wiln/stargazers) [Weather Invariant Lidar-based Navigation (WILN)](https://github.com/norlab-ulaval/wiln): 
  - A lidar-based Teach-and-Repeat framework designed to enable outdoor autonomous navigation in harsh weather. [Norlab](https://norlab.ulaval.ca) is maintaining and using the framework to deploy mobile robots in harsh weather, recently featured in [Kilometer-scale autonomous navigation in subarctic forests: challenges and lessons learned](https://norlab.ulaval.ca/publications/field-report-ltr/).

## Datasets

- [The Boreas Dataset](https://www.boreas.utias.utoronto.ca/#/): The Boreas dataset was collected by driving a repeated route over the course of 1 year resulting in stark seasonal variations. In total, Boreas contains over 350km of driving data including several sequences with adverse weather conditions such as rain and heavy snow.
  - [![GitHub Repo stars](https://img.shields.io/github/stars/utiasASRL/pyboreas?style=social)](https://github.com/utiasASRL/pyboreas/stargazers) [pyboreas](https://github.com/utiasASRL/pyboreas): Devkit for the Boreas autonomous driving dataset
- [The Canadian Adverse Driving Conditions Dataset](http://cadcd.uwaterloo.ca/): The CADC dataset, collected during winter within the Region of Waterloo, Canada, aims to promote research to improve self-driving in adverse weather contditions.
- [The Montmorency dataset](https://norlab.ulaval.ca/research/montmorencydataset/): The dataset contains the ground truth species, diameter at breast height (DBH) and position of more than 1000 trees across four forests, as well as 11 trajectories of a lidar-equipped robot going through these forests.
- [The Montmorency Forest Wintertime Dataset](https://github.com/norlab-ulaval/Norlab_wiki/wiki/Montmorency-Forest-Wintertime-Dataset): The dataset was collected in the Montmorency subarctic forest and presents fluctuating weather, including light and heavy snow, rain, and drizzle. It contains 18.8km of autonomous navigation in a teach-and-repeat mode.
- [Precise Synthetic Image and LiDAR Dataset for Autonomous Vehicle Perception (presil)](https://uwaterloo.ca/waterloo-intelligent-systems-engineering-lab/projects/precise-synthetic-image-and-lidar-presil-dataset-autonomous): The dataset contains over 50,000 instances and includes high-definition images with full resolution depth information, semantic segmentation (images), point-wise segmentation (point clouds), ground point labels (point clouds), and detailed annotations for vehicles and people in Grand Theft Auto V (GTA V), a commercial video game.
  - [![GitHub Repo stars](https://img.shields.io/github/stars/bradenhurl/DeepGTAV-PreSIL?style=social)](https://github.com/bradenhurl/DeepGTAV-PreSIL/stargazers) [DeepGTAV-PreSIL](https://github.com/bradenhurl/DeepGTAV-PreSIL): Data generation code used to mine data from GTAV
  - [![GitHub Repo stars](https://img.shields.io/github/stars/bradenhurl/PreSIL-tools?style=social)](https://github.com/bradenhurl/PreSIL-tools/stargazers) [PreSIL-tools](https://github.com/bradenhurl/PreSIL-tools): Scripts for generating ground planes, splits, and visualizations from the data

## Companies

### Robot manufacturers

Companies with their primary mission focusing on robotics.
You need at least 10 employees to be listed here.
The list is sorted by alphabetical order.

- Avidbots
- Bluewrist: [Official page](https://bluewrist.com/)
- Cellula Robotics
- Clearpath: [Official page](https://clearpathrobotics.com/), [GitHub](https://github.com/clearpathrobotics)
- Kinova: [Official page](https://www.kinovarobotics.com/), [GitHub](https://github.com/Kinovarobotics)
- Robotiq

### Natural Resources

- MacLean Engineering: [Official page](https://macleanengineering.com)
- Rigid Robotics: [Official page](https://rigidrobotics.com/)

## Organisations and Divisions

Teams or divisions working on robotics within a larger organization.

- Hydro-Québec - [Robotics team at IREQ](http://www.hydroquebec.com/robotics)
- General Dynamics Land System Canada - Innovation Cell
- LeddarTech
- FPInnovations
- MDA

## Warehouse Robotics

- [OTTO Motors](https://ottomotors.com/)
- [Think Logistics](https://www.thinklogistics.com/what-we-do/)

## Laboratories

To be on this list, the director has to have at least 1000 citations on Google Scholar.
The list is sorted by alphabetical order.
<!-- - NOTE: add keywords to lab, it might be hard categorize them as they cover mulitple topics -->

<!-- - CNRC -->

### Alberta
- [Intelligent Robot Learning Laboratory](https://irll.ca/team/), University of Alberta:
  - Fundamental reinforcement learning research, Application of artificial intelligence to real-world settings
  - Director: [Matthew E. Taylor](https://scholar.google.ca/citations?user=edQgLXcAAAAJ&hl=en&oi=ao)
- [Robotarium Laboratory](https://www.uvs-robotarium-lab.ca/), University of Calgary:
  - Control, Artificial Intelligence, Navigation, Localization, Mapping, Robot Cooperation & Collaboration, Human-Robot Interaction (HRI)
  - Director: Alex Ramirez-Serrano <!-- - no google scholar profile! seems to have low citation count -->
<!-- - UAlberta -->

### British Columbia

- [Autonomous Intelligence and Robotics Lab (AIRob)](https://robotics.sfu.ca/airob.html), Simon Fraser University:
  - Automated planning, Multi-Agent/Robot Systems, Spatio-Temporal and Constraint Reasoning
  - Director: [Hang Ma](https://scholar.google.ca/citations?user=KJbsVl8AAAAJ)
- [Collaborative Advances Robotics and Intelligent Systems lab (CARIS)](https://caris.mech.ubc.ca), University of British Columbia:
  - Ethics and Human-Robot Interaction
  - Director: [Machiel Van der Loos](https://scholar.google.ca/citations?user=ze-QdW0AAAAJ)
- [Multi-Agent Robotic Systems Lab (MARS)](https://sfumars.com), Simon Fraser University:
  - Robotic safety, Reinforcement learning, Human intent inference, Visual navigation
  - Director: [Mo Chen](https://scholar.google.ca/citations?user=19UAgLUAAAAJ)
- [UBC Robotics and Control Laboratory](https://rcl.ece.ubc.ca/home-page/), University of British Columbia:
  - Medical image analysis, Image guided diagnosis and interventions, Telerobotic, Robotic control of mobile machines and manipulators
  - Directors: [Purang Abolmaesumi](https://scholar.google.ca/citations?user=gKZS5-IAAAAJ), [Tim Salcudean](https://scholar.google.ca/citations?user=kpp_A9wAAAAJ), [Robert Rohling](https://scholar.google.ca/citations?user=_KwSTGIAAAAJ)


<!-- More UBC ? -->

<!-- UP COMING PROFESSORS:
- [ROSIE Lab](https://www.rosielab.ca), Simon Fraser University:
  - Social Intelligence, Empathy, Human-Robot Interaction
  - Director: [Angelica Lim](https://scholar.google.ca/citations?user=1jQEmPUAAAAJ)
-->


<!-- ### Manitoba -->

<!-- UManitoba -->

### Ontario

- [Advanced Robotics and Intelligent Systems Laboratory (ARIS)](https://syang.uoguelph.ca/aris/), University of Guelph:
  - Intelligent Systems, Robotics, Control Systems, Vision and Signal Processing, Sensors, Communications
  - Director: [Simon X. Yang](https://scholar.google.com/citations?hl=en&user=cyRDiPMAAAAJ)
- [Autonomous Systems and Biomechatronics Lab (ASB Lab)](http://asblab.mie.utoronto.ca), University of Toronto:
  - Assistive and Social Robotics, Search and Rescue, Intelligent Robotics, 3D Sensing, Human-Robot Interaction
  - Director: [Goldie Nejat](https://scholar.google.com/citations?hl=en&user=1pCgjH0AAAAJ)
- [Autonomous Systems Laboratory (ASL)](https://uwaterloo.ca/autonomous-systems-lab), University of Waterloo:
  - Robot Motion Planning, Future Transportation Systems, Autonomous Driving
  - Director: [Stephen L. Smith](https://scholar.google.com/citations?user=_gfwCNwAAAAJ)
- [Autonomous Space Robotics Lab (ASRL)](http://asrl.utias.utoronto.ca), University of Toronto:
  - Mobile robots, Vision-based navigation, [GitHub](https://github.com/utiasASRL)
  - Director: [Timothy D Barfoot](https://scholar.google.ca/citations?user=N_vPIhoAAAAJ)
- [Autonomous Vehicle Research and Intelligence Lab (AVRIL)](https://uwaterloo.ca/autonomous-vehicle-research-intelligence-lab), University of Waterloo:
  - Passenger Vehicles,  Automated driving,  Connected (V2X) Vehicles,  ADAS,  Driverless shuttles, Commercial trucks, Warehouse and industrial robots, Field platform vehicles
  - Directors: [Amir Khajepour](https://scholar.google.ca/citations?user=TL84ll0AAAAJ), [John McPhee](https://scholar.google.com/citations?user=boJMp7AAAAAJ)
- [Offroad Robotics](https://offroad.engineering.queensu.ca/), Queen's University:
  - Field and mobile robotics, state estimation, mechatronics, and systems control, [GitHub](https://github.com/offroad-robotics)
  - Director: [Josh Marshall](https://scholar.google.ca/citations?user=2zOIvcUAAAAJ)
- [People, AI, & Robots lab (PAIR)](https://www.pair.toronto.edu/):
  - Robotic Manipulation, Robot Learning, Reinforcement Learning, Machine Learning, Computer Vision
  - Director: [Animesh Garg](https://scholar.google.com/citations?hl=en&user=zp8V7ZMAAAAJ)
- [Space and Terrestrial Autonomous Robotic Systems (STARS Laboratory)](https://starslab.ca/), University of Toronto:
  - Collaborative Robotics, Mobile Manipulation, Multimodal Sensing, Computer Vision, Machine Learning, [GitHub](https://github.com/utiasSTARS)
  - Director: [Jonathan Kelly](https://scholar.google.ca/citations?user=KtSR8_0AAAAJ)
- [Toronto Robotics and AI Laboratory (TRAILab)](https://www.trailab.utias.utoronto.ca/), University of Toronto:
  - 3D Object Detection, Drone Landing, Robotic Manipulation, Robotics perception, Planning problems, [GitHub](https://github.com/TRAILab)
  - Previously the WAVE Lab in University of Waterloo ([GitHub](https://github.com/wavelab))
  - Director: [Steven Waslander](https://scholar.google.com/citations?user=jY_Bcd8AAAAJ)
- [Waterloo Intelligent Systems Engineering Lab (WISE Lab)](https://uwaterloo.ca/waterloo-intelligent-systems-engineering-lab), University of Waterloo
  - Autonomous Driving, Human Driving Behaviour Modeling, Safety And Quality Requirements, Machine Learning, Reinforcement Learning, Deep Learning, Simulation, [GitLab](https://git.uwaterloo.ca/wise-lab)
  - Director: [Krzysztof Czarnecki](https://scholar.google.ca/citations?user=ZzCpumQAAAAJ)

<!-- This is a center, not a lab:
- [Waterloo Centre for Automotive Research (WATCAR)](https://uwaterloo.ca/centre-automotive-research)), University of Waterloo:
  - Autonomous driving, advanced powertrains, [Autonomoose](https://www.autonomoose.net/) autonomous car
  - Director: Sebastian Fischmeister 
-->

<!-- UOttawa -->
<!-- York University -->
<!-- McMaster University -->
<!-- Ryerson University -->

<!-- UP COMING PROFESSORS:
-->

<!-- ### Prince Edward Island -->

<!-- - UPEI -->

### Quebec

<!-- [Intelligent machine Lab] -->
<!-- USherbrooke -->
<!-- UMontreal -->
<!-- UQAM - ETS -->
<!-- McGill -->
<!-- Concordia University -->
<!-- INRS -->

- [Aerospace Mechatronics Laboratory (AML)](http://aerospacemechatronics.lab.mcgill.ca/), McGill:
  - UAVs, Space robotics systems, Timber grasping
  - Director: [Inna Sharf](https://scholar.google.ca/citations?user=Ys2mNPkAAAAJ)
- [Control and Robotics Laboratory (CoRo)](https://en.etsmtl.ca/unites-de-recherche/coro/accueil?lang=en-CA), École de technologie supérieure (ÉTS):
  - Precision robotics, parallel robotics, mechatronics and haptics, control
  - Director: [Ilian Bonev](https://scholar.google.ca/citations?hl=en&user=YdVfbUgAAAAJ)
- [Intelligent / Interactive / Integrated / Interdisciplinary Robot Lab (IntRoLab)](https://introlab.3it.usherbrooke.ca), Université de Sherbrooke:
  - Service Robots, All-Terrain Robots, Interactive Robots, Assistive Robotics, Tele-Health Robotics, Automobile, Surgical Robots
  - Director: [Francois Michaud](https://scholar.google.com/citations?user=CHlw77gAAAAJ)
- [Laboratoire de robotique](https://robot.gmc.ulaval.ca/en/home), Université Laval:
  - Parallel mechanisms, Articulated robotic hands, Cable-driven parallel mechanisms, Physical human-robot interaction, Haptic devices, Assistive devices
  - Director: [Clément Gosselin](https://scholar.google.ca/citations?user=O9VSIMoAAAAJ)
- [Mobile Robotics Lab (MRL)](https://www.cim.mcgill.ca/~mrl/), McGill
  - Perception, planning, underwater robotics
  - Directors: [Gregory Dudek](https://scholar.google.com/citations?hl=en&user=BSORuFoAAAAJ)
- [Mobile Robotics and Autonomous Systems Laboratory (MRASL)](https://www.polymtl.ca/robotique-mobile/en), Polytechnique Montréal:
  - Perception, control, real-time planning and decision-making under uncertainty
  - Director: [Jerome Le Ny](https://scholar.google.ca/citations?user=YCr3i00AAAAJ)
- [Northern Robotics Laboratory (Norlab)](https://norlab.ulaval.ca), Université Laval:
  - Autonomous vehicles, lidars, SLAM, and control, [GitHub](https://github.com/norlab-ulaval)
  - Director: [François Pomerleau](https://scholar.google.ca/citations?user=FJ9IqNoAAAAJ&hl=en)
- [Robotics and Embodied AI Lab (REAL)](https://montrealrobotics.ca/), Université de Montréal:
  - Robotics and Embodied AI
  - Director: [Liam Paull](https://montrealrobotics.ca/)


<!-- UP COMING PROFESSORS:
- [INIT Robots](https://initrobots.etsmtl.ca/), ETS Montréal
  - Human-robot interaction, teleoperation
  - Director: [David St-Onge](https://scholar.google.com/citations?hl=en&user=1_ytQ1YAAAAJ)
- [RAISE Lab](https://raise.cim.mcgill.ca), McGill:
  - Ethics and Human-Robot Interaction
  - Director: [AJung Moon](https://scholar.google.ca/citations?user=NNnTnK8AAAAJ)
- [Aerospace Robotics Laboratory](https://users.encs.concordia.ca/~kskoniec/), Concordia University:
  - Director: [Krzysztof (Chris) Skonieczny](https://scholar.google.ca/citations?hl=en&user=WOSCmiEAAAAJ)
-->

<!-- This is a center, not a lab:
- [Institut interdisciplinaire d'innovation technologique (3IT)](https://www.usherbrooke.ca/3it), Université de Sherbrooke:
  - Applications of robotics in society, Drones
  - Research groups:
    - [Collaborative Robotics Technologies in Manufacturing (CoRoM)](https://corom.ca): NSERC-CREATE program with ETS and Université Laval
    - [Createk](https://www.createk.co) : mechanical engineering research group
  - Director: [Richard Arès](https://scholar.google.ca/citations?user=6IorTL0AAAAJ)
-->
### New Brunswick

- [Robotics and Mechanisms Laboratory](https://www2.unb.ca/ram/links/links.html), University of New Brunswick:
  - Parallel manipulators, Simulation of robotic systems
  - Director: [Juan A. Carretero](https://scholar.google.ca/citations?user=dGVqmGAAAAAJ)
<!-- - UNB Robotics -->

### Nova Scotia

- [Advanced Control and Mechatronics Laboratory](http://acm.me.dal.ca/index.htm), Dalhousie University:
  - Robust Nonlinear Control, Teleoperation Systems, Multi-Agent Systems Cooperation, Intelligent Transportation Control,Haptics, Exoskeletons, Human Machine Interfaces, Model Predictive Control, Assistive Robotics
  - Director: [Ya-Jun Pan](https://scholar.google.ca/citations?user=Ptjotv8AAAAJ)

<!-- - Dalhouse University -->


<!-- ### Newfoundland -->

<!-- Memorial University -->

## Related Awesome Lists

- [Awesome LIDAR](https://github.com/szenergy/awesome-lidar/)
- [Awesome Robotics Libraries](http://jslee02.github.io/awesome-robotics-libraries/)
- [Awesome Robotics](https://github.com/ahundt/awesome-robotics#point-clouds)
