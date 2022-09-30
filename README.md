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
- [Organisations and Divisions](#organisations-and-divisions)
- [Laboratories](#laboratories)
- [Related Awesome Lists](#related-awesome-lists)

## Open-Source Software

You need at least 15 stars on your repository to be listed here.
The list is sorted by number of stars.

- [![GitHub Repo stars](https://img.shields.io/github/stars/ethz-asl/libpointmatcher?style=social)](https://github.com/ethz-asl/libpointmatcher/stargazers) [libpointmatcher](https://github.com/ethz-asl/libpointmatcher): An Iterative Closest Point (ICP) library for 2D and 3D mapping in Robotics. [Norlab](https://norlab.ulaval.ca) is maintaining and using the library for their research on autonomous navigation in harsh environments.
- [![GitHub Repo stars](https://img.shields.io/github/stars/norlab-ulaval/wiln?style=social)](https://github.com/norlab-ulaval/wiln/stargazers) [Weather Invariant Lidar-based Navigation (WILN)](https://github.com/norlab-ulaval/wiln): A lidar-based Teach-and-Repeat framework designed to enable outdoor autonomous navigation in harsh weather. [Norlab](https://norlab.ulaval.ca) is maintaining and using the framework to deploy mobile robots in harsh weather, recently featured in [Kilometer-scale autonomous navigation in subarctic forests: challenges and lessons learned](https://norlab.ulaval.ca/publications/field-report-ltr/).
- [![GitHub Repo stars](https://img.shields.io/github/stars/utiasASRL/vtr3?style=social)](https://github.com/utiasASRL/vtr3/stargazers) [VT&R3](https://github.com/utiasASRL/vtr3): VT&R3 is a C++ implementation of the Teach and Repeat navigation framework. It enables a robot to be taught a network of traversable paths and then closely repeat any part of the network.  [utiasASRL](https://utiasasrl.github.io/) is maintaining and using the package for their research on vision-based localization algorithms in outdoor environments.
- [![GitHub Repo stars](https://img.shields.io/github/stars/nickcharron/lidar_snow_removal?style=social)](https://github.com/nickcharron/lidar_snow_removal/stargazers) [lidar_snow_removal](https://github.com/nickcharron/lidar_snow_removal): This repo is a set of nodes for ROS to filter point clouds with the goal of removing snow in Lidar data. [TrailLAb](https://www.trailab.utias.utoronto.ca/) was maintaining this package for their publications on the [Canadian Adverse Weather Dataset](http://cadcd.uwaterloo.ca/).
- [![GitHub Repo stars](https://img.shields.io/github/stars/SherbyRobotics/pyro?style=social)](https://github.com/SherbyRobotics/pyro/stargazers) [pyro](https://github.com/SherbyRobotics/pyro): An object-based toolbox for robot dynamic simulation, analysis, control and planning. [Createk](https://www.createk.co/) and Prof. Alexandre Girard is maintaining this library for their research on robot design, control and planning.
- [![GitHub Repo stars](https://img.shields.io/github/stars/SFU-MARS/optimized_dp?style=social)](https://github.com/SFU-MARS/optimized_dp/stargazers) [optimized_dp](https://github.com/SFU-MARS/optimized_dp):  Optimizing Dynamic Programming-Based Algorithms Resources. [SFU-MARS](https://sfumars.com/research/) is maintaining this library for their research on principled robot decision making ombining traditional analytical methods in robotics and modern data-driven techniques.

## Datasets

- [The Canadian Adverse Driving Conditions Dataset](http://cadcd.uwaterloo.ca/): The CADC dataset, collected during winter within the Region of Waterloo, Canada, aims to promote research to improve self-driving in adverse weather contditions.
- [The Boreas Dataset](https://www.boreas.utias.utoronto.ca/#/): The Boreas dataset was collected by driving a repeated route over the course of 1 year resulting in stark seasonal variations. In total, Boreas contains over 350km of driving data including several sequences with adverse weather conditions such as rain and heavy snow.
  - [![GitHub Repo stars](https://img.shields.io/github/stars/utiasASRL/pyboreas?style=social)](https://github.com/utiasASRL/pyboreas/stargazers) [pyboreas](https://github.com/utiasASRL/pyboreas): Devkit for the Boreas autonomous driving dataset
- [The Montmorency dataset](https://norlab.ulaval.ca/research/montmorencydataset/): The dataset contains the ground truth species, diameter at breast height (DBH) and position of more than 1000 trees across four forests, as well as 11 trajectories of a lidar-equipped robot going through these forests.
- [The Montmorency Forest Wintertime Dataset](https://github.com/norlab-ulaval/Norlab_wiki/wiki/Montmorency-Forest-Wintertime-Dataset): The dataset was collected in the Montmorency subarctic forest and presents fluctuating weather, including light and heavy snow, rain, and drizzle. It contains 18.8km of autonomous navigation in a teach-and-repeat mode.

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

## Warehouse robotics

- [OTTO Motors](https://ottomotors.com/)
- [Think Logistics](https://www.thinklogistics.com/what-we-do/)

## Laboratories

To be on this list, the director has to have at least 1000 citations on Google Scholar.
The list is sorted by alphabetical order.
<!-- - NOTE: add keywords to lab, it might be hard categorize them as they cover mulitple topics -->

<!-- - CNRC -->

### Alberta

- [Robotarium Laboratory](https://www.uvs-robotarium-lab.ca/), University of Calgary:
  - Control, Artificial Intelligence, Navigation, Localization, Mapping, Robot Cooperation & Collaboration, Human-Robot Interaction (HRI)
  - Director: Alex Ramirez-Serrano <!-- - no google scholar profile! seems to have low citation count -->
<!-- - UAlberta -->

### British Columbia

- [CARIS Lab](https://caris.mech.ubc.ca), University of British Columbia:
  - Ethics and Human-Robot Interaction
  - Director: [Machiel Van der Loos](https://scholar.google.ca/citations?user=ze-QdW0AAAAJ)
- [UBC Robotics and Control Laboratory](https://rcl.ece.ubc.ca/home-page/), University of British Columbia:
  - Medical image analysis, Image guided diagnosis and interventions, Telerobotic, Robotic control of mobile machines and manipulators
  - Directors: [Purang Abolmaesumi](https://scholar.google.ca/citations?user=gKZS5-IAAAAJ), [Tim Salcudean](https://scholar.google.ca/citations?user=kpp_A9wAAAAJ), [Robert Rohling](https://scholar.google.ca/citations?user=_KwSTGIAAAAJ)
- [AIRob Lab](https://robotics.sfu.ca/airob.html), Simon Fraser University:
  - Automated planning, Multi-Agent/Robot Systems, Spatio-Temporal and Constraint Reasoning
  - Director: [Hang Ma](https://scholar.google.ca/citations?user=KJbsVl8AAAAJ)
- [MARS Lab](https://sfumars.com), Simon Fraser University:
  - Robotic safety, Reinforcement learning, Human intent inference, Visual navigation
  - Director: [Mo Chen](https://scholar.google.ca/citations?user=19UAgLUAAAAJ)
- [ROSIE Lab](https://www.rosielab.ca), Simon Fraser University:
  - Social Intelligence, Empathy, Human-Robot Interaction
  - Director: [Angelica Lim](https://scholar.google.ca/citations?user=1jQEmPUAAAAJ)

<!-- More UBC ? -->


### Manitoba

<!-- UManitoba -->

### Ontario

- [Offroad Robotics](https://offroad.engineering.queensu.ca/), Queen's University:
  - Field and mobile robotics, state estimation, mechatronics, and systems control, [GitHub](https://github.com/offroad-robotics)
  - Director: TODO
- Advanced Robotics and Intelligent Systems Laboratory ([ARIS](https://syang.uoguelph.ca/aris/)), University of Guelph:
  - Intelligent Systems, Robotics, Control Systems, Vision and Signal Processing, Sensors, Communications
  - Director: TODO
- Autonomous Systems and Biomechatronics Lab ([ASB Lab](http://asblab.mie.utoronto.ca)), University of Toronto:
  - Ethics and Human-Robot Interaction
  - Director: TODO
- Autonomous Space Robotics Lab ([ASRL](http://asrl.utias.utoronto.ca)), University of Toronto:
  - Mobile robots, Vision-based navigation, [GitHub](https://github.com/utiasASRL)
  - Director: [Timothy D Barfoot](https://scholar.google.ca/citations?user=N_vPIhoAAAAJ)
- Space and Terrestrial Autonomous Robotic Systems ([STARS Laboratory](https://starslab.ca/)), University of Toronto:
  - TODO keywords, [GitHub](https://github.com/utiasSTARS)
  - Director: [Jonathan Kelly](https://scholar.google.ca/citations?user=KtSR8_0AAAAJ)

<!-- UOttawa -->
<!-- York University -->
<!-- McMaster University -->
<!-- Ryerson University -->

### Prince Edward Island

<!-- - UPEI -->

### Quebec

<!-- [Intelligent machine Lab] -->
<!-- USherbrooke -->
<!-- UMontreal -->
<!-- UQAM - ETS -->
<!-- McGill -->
<!-- INRS -->
- [Control and Robotics Laboratory (CoRo)](https://en.etsmtl.ca/unites-de-recherche/coro/accueil?lang=en-CA), École de technologie supérieure (ÉTS):
  - Precision robotics, parallel robotics, mechatronics and haptics, control
  - Director: [Ilian Bonev](https://en.etsmtl.ca/Bottin/ETS/Alphabetique/FicheEmploye?Numero=2363)
- [INIT Robots](https://initrobots.etsmtl.ca/), ETS Montréal
  - Human-robot interaction, teleoperation
  - Director: David St-Onge
- [Laboratoire de robotique](https://robot.gmc.ulaval.ca/en/home), Université Laval:
  - Parallel mechanisms, Articulated robotic hands, Cable-driven parallel mechanisms, Physical human-robot interaction, Haptic devices, Assistive devices
  - Director: [Clément Gosselin](https://scholar.google.ca/citations?user=O9VSIMoAAAAJ)
- Northern Robotics Laboratory ([Norlab](https://norlab.ulaval.ca)), Université Laval:
  - Autonomous vehicles, lidars, SLAM, and control, [GitHub](https://github.com/norlab-ulaval)
  - Director: [François Pomerleau](https://scholar.google.ca/citations?user=FJ9IqNoAAAAJ&hl=en)
- [Mobile Robotics and Autonomous Systems Laboratory](https://www.polymtl.ca/robotique-mobile/en), Polytechnique Montréal:
  - Perception, control, real-time planning and decision-making under uncertainty
  - Director: [Jerome Le Ny](https://scholar.google.ca/citations?user=YCr3i00AAAAJ)
- Aerospace Mechatronics Laboratory ([AML](http://aerospacemechatronics.lab.mcgill.ca/)), McGill:
  - UAVs, Space robotics systems, Timber grasping
  - Director: [Inna Sharf](https://scholar.google.ca/citations?user=Ys2mNPkAAAAJ)
- [Mobile Robotics Lab (MRL)](https://www.cim.mcgill.ca/~mrl/), McGill
  - Perception, planning, underwater robotics
  - Directors: Gregory Dudek, David Meger
- [RAISE Lab](https://raise.cim.mcgill.ca), McGill:
  - Ethics and Human-Robot Interaction
  - Director: [AJung Moon](https://scholar.google.ca/citations?user=NNnTnK8AAAAJ)
- [REAL Lab](https://montrealrobotics.ca/), Université de Montréal:
  - Robotics and Embodied AI
  - Director: [Liam Paull](https://montrealrobotics.ca/)

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



### Newfoundland

<!-- Memorial University -->

## Related Awesome Lists

- [Awesome LIDAR](https://github.com/szenergy/awesome-lidar/)
- [Awesome Robotics Libraries](http://jslee02.github.io/awesome-robotics-libraries/)
- [Awesome Robotics](https://github.com/ahundt/awesome-robotics#point-clouds)
