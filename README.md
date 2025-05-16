<!--lint disable double-link-->
# Awesome Canadian Robotics [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

> A curated list of Canadian robotics open-source software, datasets, companies, laboratories, and researchers.

The maintenance of this list is aligned with the proposition of the [Canadian Robotics Council](https://www.roboticscouncil.ca) to better share information about robotics initiative in Canada.
This is not a comprehensive list of everything happening in Canada related to robotics.
We added abitrary criterias to be listed here with the intention to be transparent.

## Contents

- [Open-Source Software](#open-source-software)
- [Datasets](#datasets)
- [Companies](#companies)
  - [Robot manufacturers](#robot-manufacturers)
  - [Aerospace](#aerospace)
  - [Natural Resources](#natural-resources)
  - [Warehouse Robotics](#warehouse-robotics)
- [Organisations and Divisions](#organisations-and-divisions)
- [Laboratories](#laboratories)
  - [Alberta](#alberta)
  - [British Columbia](#british-columbia)
  - [Ontario](#ontario)
  - [Quebec](#quebec)
  - [New Brunswick](#new-brunswick)
  - [Nova Scotia](#nova-scotia)
  - [Newfoundland](#newfoundland)
- [Related Lists](#related-lists)
  - [Awesome Lists](#awesome-lists)
  - [Canadian Robotics Research Lists](#canadian-robotics-research-lists)

## Open-Source Software

You need at least 15 stars on your repository to be listed here.
The list is sorted by number of stars.


- [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) - An Iterative Closest Point (ICP) library for 2D and 3D mapping in Robotics in C++. [Norlab](https://norlab.ulaval.ca) is maintaining and using the library for their research on autonomous navigation in harsh environments. <p align="right">[![GitHub Repo stars](https://img.shields.io/github/stars/ethz-asl/libpointmatcher?style=social)](https://github.com/ethz-asl/libpointmatcher/stargazers)</p>

- [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) - Official ROS packages to interface with Kinova's KORTEX platform, supporting control, feedback, and example applications for Kinova Gen3 and Gen3 Lite robotic arms. Developed and maintained by [Kinova Robotics](https://www.kinovarobotics.com).  <p align="right">[![GitHub Repo stars](https://img.shields.io/github/stars/Kinovarobotics/ros_kortex?style=social)](https://github.com/Kinovarobotics/ros_kortex/stargazers)</p>


- [VT&R3](https://github.com/utiasASRL/vtr3) - A C++ implementation of the Teach and Repeat navigation framework. It enables a robot to be taught a network of traversable paths and then closely repeat any part of the network.  [utiasASRL](https://utiasasrl.github.io) is maintaining and using the package for their research on vision-based localization algorithms in outdoor environments. <p align="right">[![GitHub Repo stars](https://img.shields.io/github/stars/utiasASRL/vtr3?style=social)](https://github.com/utiasASRL/vtr3/stargazers)</p>

- [optimized_dp](https://github.com/SFU-MARS/optimized_dp) - Optimizing Dynamic Programming-Based Algorithms Resources. [SFU-MARS](https://sfumars.com/research) is maintaining this library for their research on principled robot decision making ombining traditional analytical methods in robotics and modern data-driven techniques. <p align="right">[![GitHub Repo stars](https://img.shields.io/github/stars/SFU-MARS/optimized_dp?style=social)](https://github.com/SFU-MARS/optimized_dp/stargazers)</p>

- [lidar_snow_removal](https://github.com/nickcharron/lidar_snow_removal) - A set of nodes for ROS to filter point clouds with the goal of removing snow in Lidar data. [TrailLAb](https://www.trailab.utias.utoronto.ca) was maintaining this package for their publications on the [Canadian Adverse Weather Dataset](http://cadcd.uwaterloo.ca). <p align="right">[![GitHub Repo stars](https://img.shields.io/github/stars/nickcharron/lidar_snow_removal?style=social)](https://github.com/nickcharron/lidar_snow_removal/stargazers)</p>

- [norlab_icp_mapper](https://github.com/norlab-ulaval/norlab_icp_mapper) - A 2-D/3-D mapping library relying on the "Iterative Closest Point" algorithm. [Norlab](https://norlab.ulaval.ca) is maintaining and using the framework to deploy mobile robots in extreme conditions, recently featured in [Kilometer-scale autonomous navigation in subarctic forests: challenges and lessons learned](https://norlab.ulaval.ca/publications/field-report-ltr) and [Lidar Scan Registration Robust to Extreme Motions](https://norlab.ulaval.ca/publications/extreme-motions). <p align="right">[![GitHub Repo stars](https://img.shields.io/github/stars/norlab-ulaval/norlab_icp_mapper?style=social)](https://github.com/norlab-ulaval/norlab_icp_mapper/stargazers)</p>

- [pyro](https://github.com/SherbyRobotics/pyro) - An object-based toolbox for robot dynamic simulation, analysis, control and planning. Developed by USherbrooke's [Createk](https://www.createk.co) for their research on dynamic systems' design, control, simulation and planning. <p align="right">[![GitHub Repo stars](https://img.shields.io/github/stars/SherbyRobotics/pyro?style=social)](https://github.com/SherbyRobotics/pyro/stargazers)</p>

- [clearpath_common](https://github.com/clearpathrobotics/clearpath_common) - A collection of ROS packages and configuration files maintained by [Clearpath Robotics](https://clearpathrobotics.com) to support their fleet of mobile robots for research and industrial applications. <p align="right">[![GitHub Repo stars](https://img.shields.io/github/stars/clearpathrobotics/clearpath_common?style=social)](https://github.com/clearpathrobotics/clearpath_common/stargazers)</p>

- [mrasl_mav_traj](https://github.com/MRASL/mrasl_mav_traj) - Trajectory utilities for Micro UAVs (MAVs). Maintained by the lab [MRASL](http://www.polymtl.ca/robotique-mobile/en). <p align="right">[![GitHub Repo stars](https://img.shields.io/github/stars/MRASL/mrasl_mav_traj?style=social)](https://github.com/MRASL/mrasl_mav_traj/stargazers)</p>

- [Weather Invariant Lidar-based Navigation (WILN)](https://github.com/norlab-ulaval/wiln) - A lidar-based Teach-and-Repeat framework designed to enable outdoor autonomous navigation in harsh weather. [Norlab](https://norlab.ulaval.ca) is maintaining and using the framework to deploy mobile robots in harsh weather, recently featured in [Kilometer-scale autonomous navigation in subarctic forests: challenges and lessons learned](https://norlab.ulaval.ca/publications/field-report-ltr). <p align="right">[![GitHub Repo stars](https://img.shields.io/github/stars/norlab-ulaval/wiln?style=social)](https://github.com/norlab-ulaval/wiln/stargazers) </p>

<!-- NOT ENOUGH STARS for now
- [![GitHub Repo stars](https://img.shields.io/github/stars/norlab-ulaval/PercepTreeV1?style=social)]([[https://github.com/norlab-ulaval/PercepTreeV1/stargazers](https://github.com/norlab-ulaval/PercepTreeV1)](https://github.com/norlab-ulaval/PercepTreeV1)) [Tree detection based on deep learning](https://github.com/norlab-ulaval/PercepTreeV1 (PercepTree)):
  - An image-based solution to detect trees in outdoor environments. [Norlab](https://norlab.ulaval.ca) is maintaining and using the framework to develop autonomous precision forestry applications. <p align="right">TMP</p>
-->

## Datasets

- [The Boreas Dataset](https://www.boreas.utias.utoronto.ca) - The Boreas dataset was collected by driving a repeated route over the course of 1 year resulting in stark seasonal variations. In total, Boreas contains over 350km of driving data including several sequences with adverse weather conditions such as rain and heavy snow.
  - [pyboreas](https://github.com/utiasASRL/pyboreas) - Devkit for the Boreas autonomous driving dataset. [![GitHub Repo stars](https://img.shields.io/github/stars/utiasASRL/pyboreas?style=social)](https://github.com/utiasASRL/pyboreas/stargazers)
- [The Canadian Adverse Driving Conditions Dataset](http://cadcd.uwaterloo.ca) -  The CADC dataset, collected during winter within the Region of Waterloo, Canada, aims to promote research to improve self-driving in adverse weather conditions.
- [Leddar PixSet Dataset](https://leddartech.com/solutions/leddar-pixset-dataset/) - The First Full-Waveform Flash LIDAR Dataset for Autonomous Vehicle R&D. The PixSet dataset contains 97 sequences for a total of roughly 29k frames using the AV sensor suite. Each frame has been manually annotated with 3D bounding boxes. The sequences have been gathered in various Canadian environments (e.g., urban, suburban like highway), climatic (e.g., sunny, cloudy, rainy) conditions and illumination (e.g., day, night, twilight) conditions with an instrumented vehicle.
- [The Canadian Planetary Emulation Terrain Energy-Aware Rover Navigation (`enav-planetary`) Dataset](https://starslab.ca/enav-planetary-dataset) - Developed by UToronto's STARS Lab, the Energy-Aware Planetary Navigation Dataset has 1.2 km of data from a typical rover's sensor payload. The goal of this dataset is to promote rover energy management strategies for future exploratory missions to the Moon and Mars. Introduced in [The Canadian Planetary Emulation Terrain Energy-Aware Rover Navigation Dataset](https://doi.org/10.1177/0278364920908922).
  - [enav-planetary-dataset](https://github.com/utiasSTARS/enav-planetary-dataset) - ROS packages to visualize and interact with the data in RViz, helper scripts in Python to fetch and plot data from the rosbags. [![GitHub Repo stars](https://img.shields.io/github/stars/utiasSTARS/enav-planetary-dataset?style=social)](https://github.com/utiasSTARS/enav-planetary-dataset/stargazers)
- [The Montmorency dataset](https://norlab.ulaval.ca/research/montmorencydataset) - The dataset contains the ground truth species, diameter at breast height (DBH) and position of more than 1000 trees across four forests, as well as 11 trajectories of a lidar-equipped robot going through these forests.
- [The Montmorency Forest Wintertime Dataset](https://github.com/norlab-ulaval/Norlab_wiki/wiki/Montmorency-Forest-Wintertime-Dataset) - The dataset was collected in the Montmorency subarctic forest and presents fluctuating weather, including light and heavy snow, rain, and drizzle. It contains 18.8km of autonomous navigation in a teach-and-repeat mode.
- [Precise Synthetic Image and LiDAR Dataset for Autonomous Vehicle Perception (presil)](https://uwaterloo.ca/waterloo-intelligent-systems-engineering-lab/projects/precise-synthetic-image-and-lidar-presil-dataset-autonomous) - The dataset contains over 50,000 instances and includes high-definition images with full resolution depth information, semantic segmentation (images), point-wise segmentation (point clouds), ground point labels (point clouds), and detailed annotations for vehicles and people in Grand Theft Auto V (GTA V), a commercial video game.
  - [DeepGTAV-PreSIL](https://github.com/bradenhurl/DeepGTAV-PreSIL) - Data generation code used to mine data from GTAV. [![GitHub Repo stars](https://img.shields.io/github/stars/bradenhurl/DeepGTAV-PreSIL?style=social)](https://github.com/bradenhurl/DeepGTAV-PreSIL/stargazers)
  - [PreSIL-tools](https://github.com/bradenhurl/PreSIL-tools) - Scripts for generating ground planes, splits, and visualizations from the data. [![GitHub Repo stars](https://img.shields.io/github/stars/bradenhurl/PreSIL-tools?style=social)](https://github.com/bradenhurl/PreSIL-tools/stargazers)
- [Forest image (PercepTree) datasets](https://github.com/norlab-ulaval/PercepTreeV1) - This repository contains two datasets: a 43,000 synthetic forest images and a 100 real image dataset. Both include high-definition RGB images with depth information, bounding box, instance segmentation masks and keypoints annotation. [![GitHub Repo stars](https://img.shields.io/github/stars/norlab-ulaval/PercepTreeV1?style=social)]([[https://github.com/norlab-ulaval/PercepTreeV1/stargazers](https://github.com/norlab-ulaval/PercepTreeV1)](https://github.com/norlab-ulaval/PercepTreeV1))
- [University of Toronto Indoor 3D Dataset](https://github.com/utiasASRL/UTIn3D) - This repository contains a robot navigation dataset in crowded indoor environment. It includes the lidar frames, their localization computed by ICP based algorithm PointMap, and the labels provided by automated annotation approach. It was introduced in [Learning Spatiotemporal Occupancy Grid Maps for Lifelong Navigation in Dynamic Scenes](https://arxiv.org/pdf/2108.10585.pdf). ![GitHub Repo stars](https://img.shields.io/github/stars/utiasASRL/UTIn3D?style=social)

## Companies

Companies with their primary mission focusing on robotics and [registered in Canada](https://beta.canadasbusinessregistries.ca/search).
You need at least ten (10) employees and be in operation for at least five (5) years to be listed here.
The list is sorted by alphabetical order.

### Robot manufacturers

- [Avidbots](https://avidbots.com) - Designs, manufactures, sells, services, and supports autonomous cleaning robots.
- [Bluewrist](https://bluewrist.com) - Develops industrial automation solutions and products in the areas of robotics and machine vision, including robot guidance, bin-picking, flexible inspection, 3D scanning and robot calibration.
- [Cellula Robotics](https://www.cellula.com) - Designs and produces subsea robotic systems.
- [Cyberworks Robotics](https://www.cyberworksrobotics.com) - Develops autonomous robotic navigation systems for mobility aids, material transport, and service robots.
- [Clearpath](https://clearpathrobotics.com) - Designs, produces, and distributes robotic systems. They have an active [GitHub repo](https://github.com/clearpathrobotics).
- [FingerTech Robotics](https://www.fingertechrobotics.com) - Designs and manufactures robot parts.
- [Kinova](https://www.kinovarobotics.com) - Designs, manufactures, sells, services, and supports robotic arms. They have an active [GitHub repo](https://github.com/Kinovarobotics).
- [IndroRobotics](https://indrorobotics.ca) - Is a UAV, ground and sea robotics research and development company.
- [Marginally Clever Robots, Ltd.](https://www.marginallyclever.com) - Through education and thought leadership, we empower people to choose STEM/STEAM careers, build robots, and solve their problems sustainably.
- [Robotiq](https://robotiq.com) - Designs, manufactures, sells, services, and supports robotic grippers.

### Aerospace

- [Canadensys Aerospace Corporation](https://www.canadensys.com) - Develops micro and nano space technology and smart, ruggedized vehicles.
- [MDA](https://mda.space) - Develops and produces all Canadarms and Dextre, a versatile robot that maintains the International Space Station.
- [Mission Control Space Services](https://www.missioncontrolspaceservices.com)
- [NGC Aerospace](https://www.ngcaerospace.com)

### Natural Resources

- [MacLean Engineering](https://macleanengineering.com)
- [Rigid Robotics](https://rigidrobotics.com)

### Warehouse Robotics

- [OTTO Motors](https://ottomotors.com)
- [Sir Steward](https://sirsteward.com) - Designs and manufactures delivery robots, smart lockers and self-serve kiosks.
- [Think Logistics](https://www.thinklogistics.com)

## Organisations and Divisions

Teams, divisions, or subsidaiary working on robotics within a larger organization.

- [Hydro-Québec Research Institute (IREQ)](http://www.hydroquebec.com/robotics) - Has a team specialized in autonomous systems for power industries.
- [General Dynamics Land System Canada](https://firstsourcerequest.gdls.com) - Has a team specialized in autonomy.
- [LeddarTech](https://leddartech.com) - Has a team focusing on autonomous driving solutions.
- [FPInnovations](https://web.fpinnovations.ca/forest-operations-solutions-to-help-the-canadian-forest-industry/forestry-4-0/) - Has a team specialized in the automation of forestry activities.
- [Institut du véhicule innovant (IVI)](https://www.ivisolutions.ca)
- [Area X.O](https://areaxo.com)
- [Provectus Robotics Solutions Inc.](https://www.rheinmetall.ca/en/rheinmetall_canada/systemsandproducts/electronicsystems/unmanned_vehicles/index.php) - A subsidaiary of Rheinmetall. Provides autonomous navigation technology that helps solve their safety, labour, and budgetary challenges.


## Laboratories

To be on this list, the director has to have at least 1000 citations on Google Scholar.
The list is sorted by alphabetical order.
<!-- - NOTE: add keywords to lab, it might be hard categorize them as they cover mulitple topics -->

<!-- - CNRC -->

### Alberta

- [Applied Nonlinear Controls Laboratory](http://ancl.ece.ualberta.ca) - From the University of Alberta and specialized in nonlinear control, analysis, prediction and influence of systems, nonlinear mathematical models, robotics, and UAVs.
  - Director: [Alan Lynch](https://scholar.google.ca/citations?user=0a_D9TgAAAAJ)
- [Artificial Intelligence and Robotics in Construction (AIRCon-Lab)](https://profsckang.wixsite.com/uofa-rlab) - From the University of Alberta and specialized in sensors, actuators, and artificial intelligence in construction sites.
  - Director: [Shih-Chung Jessy Kang](https://scholar.google.ca/citations?user=ezCU4-cAAAAJ)
- [Assistive Technology Labs](https://www.ualberta.ca/rehabilitation/research/assistive-technology-labs/index.html) - From the University of Alberta and specialized in assistive technology development, clinical research, and children's development.
  - Director: [Kim Adams](https://scholar.google.ca/citations?user=a-Qg7VgAAAAJ)
- [Bionic Limbs for Improved Natural Control](https://blinclab.ca) - From the University of Alberta and specialized in sensory motor control, integration of advanced prosthetics and robotics systems, prosthetic restoration, and rehabilitation robotics.
  - Director: [Jacqueline Hebert](https://scholar.google.ca/citations?user=Ke1MMPoAAAAJ) & [Patrick Pilarski](https://scholar.google.ca/citations?user=VGQmtWEAAAAJ)
- [Computer Vision and Robotics Group](http://webdocs.cs.ualberta.ca/~vis) - From the University of Alberta and specialized in computer vision, robotics, and medical imaging.
  - Director: [Martin Jagersand](https://scholar.google.ca/citations?user=kxAlIY4AAAAJ)
- [Intelligent Automation Research Laboratory (iAR)](https://www.ucalgary.ca/aerospace/robotics) - From the University of Calgary and specialized in industrial automation with mechatronic systems, vibration, control, optimization, and artificial intelligence.
  - Director: [Jihyun Lee](https://scholar.google.ca/citations?user=nDMhFS8AAAAJ)
- [Intelligent Robot Learning Laboratory](https://irll.ca/team) - From the University of Alberta and specialized in fundamental reinforcement learning research, application of artificial intelligence to real-world settings.
  - Director: [Matthew E. Taylor](https://scholar.google.ca/citations?user=edQgLXcAAAAJ)
- [Neuromuscular Control & Biomechanics Laboratory](https://www.ncbl.ualberta.ca) - From the University of Alberta and specialized in human mobility and performance, advanced assistive technology, and rehabilitation.
  - Director: [Hossein Rouhani](https://scholar.google.ca/citations?user=sVizzJ8AAAAJ) & [Albert H. Vette](https://scholar.google.ca/citations?user=f7w2JJoAAAAJ)
- [Robotarium Laboratory](https://www.uvs-robotarium-lab.ca) - From the University of Calgary and specialized in control, artificial intelligence, navigation, localization, mapping, robot cooperation & collaboration, and human-robot interaction (HRI).
  - Director: Alex Ramirez-Serrano <!-- - no google scholar profile! seems to have low citation count -->
- [Robotic Systems Research, Telerobotic and Biorobotic Systems Group](http://www.ece.ualberta.ca/~tbs/pmwiki) - From the University of Alberta and specialized in medical & biomedical robotics, surgery, rehabilitation, image-guided robotics, human-robot interfaces, and machine intelligence-based solutions.
  - Director: [Mahdi Tavakoli](https://scholar.google.ca/citations?user=w7XLOoEAAAAJ)
<!-- - UAlberta -->

### British Columbia

- [Autonomous Intelligence and Robotics Lab (AIRob)](https://robotics.sfu.ca/airob.html) - From the Simon Fraser University and specialized in automated planning, multi-agent/robot systems, spatio-temporal, and constraint reasoning.
  - Director: [Hang Ma](https://scholar.google.ca/citations?user=KJbsVl8AAAAJ)
- [Collaborative Advances Robotics and Intelligent Systems lab (CARIS)](https://caris.mech.ubc.ca) - From the University of British Columbia and specialized in ethics and human-robot interaction.
  - Director: [Machiel Van der Loos](https://scholar.google.ca/citations?user=ze-QdW0AAAAJ)
- [Multi-Agent Robotic Systems Lab (MARS)](https://sfumars.com) - From the Simon Fraser University and specialized in robotic safety, reinforcement learning, human intent inference, and visual navigation.
  - Director: [Mo Chen](https://scholar.google.ca/citations?user=19UAgLUAAAAJ)
- [UBC Computer Vision Lab](https://vision.cs.ubc.ca) - From the University of British Columbia and specialized in image understanding, video understanding, human pose estimation, understanding of sports videos using machine learning and deep learning techniques.
  - Director: [James J. Little](https://scholar.google.ca/citations?user=a0za4V8AAAAJ)
- [UBC Robotics and Control Laboratory](https://rcl.ece.ubc.ca/home-page) - From the University of British Columbia and specialized in medical image analysis, image guided diagnosis and interventions, telerobotic, robotic control of mobile machines, and manipulators.
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

- [Advanced Robotics and Intelligent Systems Laboratory (ARIS)](https://syang.uoguelph.ca/aris) - From the University of Guelph and specialized in intelligent systems, robotics, control systems, vision and signal processing, sensors, communications.
  - Director: [Simon X. Yang](https://scholar.google.ca/citations?user=cyRDiPMAAAAJ)
- [Autonomous Systems and Biomechatronics Lab (ASB Lab)](http://asblab.mie.utoronto.ca) - From the University of Toronto and specialized in assistive and social robotics, search and rescue, intelligent robotics, 3D sensing, human-robot interaction.
  - Director: [Goldie Nejat](https://scholar.google.ca/citations?user=1pCgjH0AAAAJ)
- [Autonomous Systems Laboratory (ASL)](https://uwaterloo.ca/autonomous-systems-lab) - From the University of Waterloo and specialized in robot motion planning, future transportation systems, and autonomous driving.
  - Director: [Stephen L. Smith](https://scholar.google.ca/citations?user=_gfwCNwAAAAJ)
- [Autonomous Space Robotics Lab (ASRL)](http://asrl.utias.utoronto.ca) - From the University of Toronto and specialized in mobile robots, and vision-based navigation. Open-source softwares available on their [GitHub repo](https://github.com/utiasASRL).
  - Director: [Timothy D Barfoot](https://scholar.google.ca/citations?user=N_vPIhoAAAAJ)
- [Autonomous Vehicle Research and Intelligence Lab (AVRIL)](https://uwaterloo.ca/autonomous-vehicle-research-intelligence-lab) - From the University of Waterloo and specialized in passenger vehicles,  automated driving,  connected (V2X) vehicles,  ADAS,  driverless shuttles, commercial trucks, warehouse and industrial robots, and field platform vehicles.
  - Directors: [Amir Khajepour](https://scholar.google.ca/citations?user=TL84ll0AAAAJ), [John McPhee](https://scholar.google.ca/citations?user=boJMp7AAAAAJ)
- [Co​ntinuum Roboti​​cs Laboratory](https://crl.utm.utoronto.ca) - From the University of Toronto and specialized in continuum robotics. Open-source softwares available on their [GitHub repo](https://github.com/ContinuumRoboticsLab).
  - Director: [Jessica Burgner-Kahrs](https://scholar.google.ca/citations?user=JtHZL24AAAAJ)
- [Lakehead University Robotics Lab](https://xpliu.lakeheadu.ca/lab) - From the Lakehead University and specialized in biped walking robots, parallel robots, nonlinear adaptive control and robust control.
  - Director: [Xiaoping Liu](https://scholar.google.ca/citations?user=H8vOyBIAAAAJ)
- [Offroad Robotics](https://offroad.engineering.queensu.ca) - From the Queen's University and specialized in field and mobile robotics, state estimation, mechatronics, and systems control. Open-source softwares available on their [GitHub repo](https://github.com/offroad-robotics).
  - Director: [Josh Marshall](https://scholar.google.ca/citations?user=2zOIvcUAAAAJ)
- [People, AI, & Robots lab (PAIR)](https://www.pair.toronto.edu) - From the University of Toronto and specialized in robotic manipulation, robot learning, reinforcement learning, machine learning, and computer vision.
  - Director: [Animesh Garg](https://scholar.google.ca/citations?user=zp8V7ZMAAAAJ)
- [Robotics & Automatic Control Laboratory](http://atayebi.lakeheadu.ca/research_group.html) - From the Lakehead University and specilalized in design of estimation and control algorithms for UAVs, vision-aided inertial navigation systems, multi-UAV collaborative applications, and GPS-denied environments.
  - Director: [Abdelhamid Tayebi](https://scholar.google.ca/citations?user=bZXTyF4AAAAJ)
- [Space and Terrestrial Autonomous Robotic Systems (STARS Laboratory)](https://starslab.ca) - From the University of Toronto and specialized in collaborative robotics, mobile manipulation, multimodal sensing, computer vision, and machine learning. Open-source softwares available on their [GitHub repo](https://github.com/utiasSTARS).
  - Director: [Jonathan Kelly](https://scholar.google.ca/citations?user=KtSR8_0AAAAJ)
- [Toronto Intelligent Systems Lab (TISL)](https://tisl.cs.utoronto.ca) - From the University of Toronto and specialized in probabilistic and geometric deep learning, 3D vision, aand reinforcement learning.
  - Director: [Igor Gilitschenski](https://scholar.google.ca/citations?user=Nuw1Y4oAAAAJ)
- [Toronto Robotics and AI Laboratory (TRAILab)](https://www.trailab.utias.utoronto.ca) - From the University of Toronto and specialized in 3D object detection, drone landing, robotic manipulation, robotics perception, and planning problems. Open-source softwares available on their [GitHub repo](https://github.com/TRAILab).
  - Director: [Steven Waslander](https://scholar.google.ca/citations?user=jY_Bcd8AAAAJ)
  - Note: This was previously the WAVE Lab in University of Waterloo ([GitHub](https://github.com/wavelab)).
- [Waterloo Intelligent Systems Engineering Lab (WISE Lab)](https://uwaterloo.ca/waterloo-intelligent-systems-engineering-lab) - From the University of Waterloo and specialized in autonomous driving, human driving behaviour modeling, safety and quality requirements, machine learning, reinforcement learning, deep learning, and  simulation. Open-source softwares available on their [GitLab repo](https://git.uwaterloo.ca/wise-lab).
  - Director: [Krzysztof Czarnecki](https://scholar.google.ca/citations?user=ZzCpumQAAAAJ)

<!-- This is a center, not a lab:
- [Waterloo Centre for Automotive Research (WATCAR)](https://uwaterloo.ca/centre-automotive-research)), University of Waterloo:
  - Autonomous driving, advanced powertrains, [Autonomoose](https://www.autonomoose.net) autonomous car
  - Director: Sebastian Fischmeister
-->

<!-- UOttawa -->
<!-- York University -->
<!-- McMaster University -->
<!-- Ryerson University -->

<!-- UP COMING PROFESSORS:
-->

<!-- ### Prince Edward Island -->

<!-- UP COMING PROFESSORS:
- [The More-Than-One Robotics Lab](https://vnbotics.blogspot.com), University of Prince Edward Island
  - Swarm Robotics
  - Director: Trung Dung Ngo
-->


### Quebec

<!-- [Intelligent machine Lab] -->
<!-- USherbrooke -->
<!-- UMontreal -->
<!-- UQAM - ETS -->
<!-- McGill -->
<!-- Concordia University -->
<!-- INRS -->

- [Aerospace Mechatronics Laboratory (AML)](http://aerospacemechatronics.lab.mcgill.ca) - From McGill University and specialized in UAVs, space robotics systems, and timber grasping.
  - Director: [Inna Sharf](https://scholar.google.ca/citations?user=Ys2mNPkAAAAJ)
- [Control and Robotics Laboratory (CoRo)](https://en.etsmtl.ca/unites-de-recherche/coro/accueil) - From École de technologie supérieure (ÉTS) and specialized in precision robotics, parallel robotics, mechatronics, haptics, and control.
  - Director: [Ilian Bonev](https://scholar.google.ca/citations?user=YdVfbUgAAAAJ)
- [Dynamics, Estimation, and Control in Aerospace and Robotics (DECAR)](https://sites.google.com/view/decarresearch/) - From McGill Univeristy and specialized in state estimation (navigation),  guidance, and control for aerial, ground, underwater and space robotic systems.
  - Director: [James Forbes](https://scholar.google.com/citations?user=PVJW68EAAAAJ&hl=en)
- [Intelligent / Interactive / Integrated / Interdisciplinary Robot Lab (IntRoLab)](https://introlab.3it.usherbrooke.ca) - From Université de Sherbrooke and specialized in service robots, all-terrain robots, interactive robots, assistive robotics, tele-health robotics, automobile, and surgical robots.
  - Director: [Francois Michaud](https://scholar.google.ca/citations?user=CHlw77gAAAAJ)
- [Laboratoire de robotique](https://robot.gmc.ulaval.ca/en/home) - From Université Laval and specialized in parallel mechanisms, articulated robotic hands, cable-driven parallel mechanisms, physical human-robot interaction, haptic devices, and assistive devices.
  - Director: [Clément Gosselin](https://scholar.google.ca/citations?user=O9VSIMoAAAAJ)
- [Mobile Robotics Lab (MRL)](https://www.cim.mcgill.ca/~mrl) - From McGill University and specialized in perception, planning, and underwater robotics. Open-source softwares available on their [GitHub repo](https://github.com/mcgillmrl).
  - Directors: [Gregory Dudek](https://scholar.google.ca/citations?user=BSORuFoAAAAJ), [David Meger](https://scholar.google.ca/citations?user=gFwEytkAAAAJ)
- [Making Innovative Space Technology (MIST)](https://mistlab.ca) - From Polytechnique Montréal and specialized in swarm robotics, and multi-robot systems. Open-source softwares available on their [GitHub repo](https://github.com/MISTLab).
  - Director: [Giovanni Beltrame](https://scholar.google.ca/citations?user=TVHJJ9wAAAAJ)
- [Mobile Robotics and Autonomous Systems Laboratory (MRASL)](https://www.polymtl.ca/robotique-mobile/en) - From Polytechnique Montréal and specialized in perception, control, real-time planning, and decision-making under uncertainty.
  - Director: [Jerome Le Ny](https://scholar.google.ca/citations?user=YCr3i00AAAAJ)
- [Northern Robotics Laboratory (Norlab)](https://norlab.ulaval.ca) - From Université Laval and specialized ini autonomous vehicles, lidars, SLAM, control, field robotics, and cryobotics. Open-source softwares available on their [GitHub repo](https://github.com/norlab-ulaval).
  - Director: [François Pomerleau](https://scholar.google.ca/citations?user=FJ9IqNoAAAAJ)
- [Robotics and Embodied AI Lab (REAL)](https://montrealrobotics.ca) - From Université de Montréal and specialized in robotics, and embodied AI.
  - Director: [Liam Paull](https://scholar.google.ca/citations?user=H9xADK0AAAAJ)


<!-- UP COMING PROFESSORS:
- [INIT Robots](https://initrobots.etsmtl.ca), ETS Montréal
  - Human-robot interaction, teleoperation
  - Director: [David St-Onge](https://scholar.google.ca/citations?user=1_ytQ1YAAAAJ)
- [RAISE Lab](https://raise.cim.mcgill.ca), McGill:
  - Ethics and Human-Robot Interaction
  - Director: [AJung Moon](https://scholar.google.ca/citations?user=NNnTnK8AAAAJ)
- [Aerospace Robotics Laboratory](https://users.encs.concordia.ca/~kskoniec), Concordia University:
  - Director: [Krzysztof (Chris) Skonieczny](https://scholar.google.ca/citations?user=WOSCmiEAAAAJ)
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

- [Robotics and Mechanisms Laboratory](https://www2.unb.ca/ram/links/links.html) - From the University of New Brunswick and specialized in parallel manipulators, and simulation of robotic systems.
  - Director: [Juan A. Carretero](https://scholar.google.ca/citations?user=dGVqmGAAAAAJ)
<!-- - UNB Robotics -->

<!-- UP COMING PROFESSORS:
- André Gallant
-->

### Nova Scotia

- [Advanced Control and Mechatronics Laboratory](http://acm.me.dal.ca/index.htm) - From Dalhousie University and specialized in robust nonlinear control, teleoperation systems, multi-agent systems cooperation, intelligent transportation control, haptics, exoskeletons, human machine interfaces, model predictive control, and assistive robotics.
  - Director: [Ya-Jun Pan](https://scholar.google.ca/citations?user=Ptjotv8AAAAJ)

<!-- - Dalhouse University -->


### Newfoundland

<!-- Moved to Bremen, Germany:
- [Autonomous Ocean Systems Laboratory](https://aosl.ca) - From Memorial University and specialized in marine robotics, autonomous underwater vehicles (AUVs), unmanned surface craft (USC)
  - Director: [Ralf Bachmayer](https://scholar.google.ca/citations?user=r-O7810AAAAJ)
-->
- [Bio-inspired Robotics (BOTS)](https://www.mun.ca/engineering/research/centres-and-facilities/research-laboratories/electrical-and-computer-engineering/bio-inspired-robotics-bots) - From Memorial University and specialized in swarm robotics.
  - Director: [Andrew Vardy](https://scholar.google.ca/citations?user=ccxPX38AAAAJ), [Todd Wareham](https://scholar.google.ca/citations?user=1cFXu-QAAAAJ), [David Churchill](https://scholar.google.ca/citations?user=teRseTgAAAAJ)
- [Intelligent Systems Laboratory](https://www.mun.ca/engineering/research/centres-and-facilities/research-laboratories/mechanical-engineering/intelligent-systems-laboratory) - From Memorial University and specialized in sensor design, state estimation, pack ice field detection and tracking, nonlinear model predictive control of mobile robots, cooperative localization of distributed multi-robotic systems, visual-inertial navigation, and robot perception using visual attention.
  - Directors: [George K. Mann](https://scholar.google.ca/citations?user=CyavUNwAAAAJ), [Ray Gosine](https://scholar.google.ca/citations?user=57WxPZMAAAAJ), [Andrew Vardy](https://scholar.google.ca/citations?user=ccxPX38AAAAJ)

<!--  This is group of labs
- [Vision, Software and Robotics Lab (VISOR)](https://www.mun.ca/engineering/research/centres-and-facilities/research-laboratories/electrical-and-computer-engineering/vision-software-and-robotics-visor-laboratory), Memorial University:
-->

<!-- Memorial University -->
<!-- UP COMING PROFESSORS:
- [Robotics, Mechanical and Control Laboratory (RoMeCoLa)](https://www.mun.ca/engineering/research/centres-and-facilities/research-laboratories/mechanical-engineering/robotics-mechanical-and-control-laboratory), Memorial University:
  - Mechanisms design, Control of Mechanica Robotic Systems
  - Director: [Ting Zou](https://scholar.google.ca/citations?user=74DtJu4AAAAJ)
-->

## Related Lists

### Awesome Lists

- [Awesome LIDAR](https://github.com/szenergy/awesome-lidar)
- [Awesome Robotics Libraries](http://jslee02.github.io/awesome-robotics-libraries)
- [Awesome Robotics](https://github.com/ahundt/awesome-robotics#point-clouds)

### Canadian Robotics Research Lists

- [Women in Academic Research in Robotics in Canada](https://canada-robotics-research-lists.github.io/canada-women-in-robotics-research/)
- [BIPOC in Academic Research in Robotics in Canada](https://canada-robotics-research-lists.github.io/canada-BIPOC-in-robotics-research/)
