# Sumpinator 2021

I was frustrated with multiple instances where our external weeping tile sump pump system failed. The builder, electrician and plumber all suggested that we simply have one sump pump and that will be a reliable solution.

We opted to have two power circuits and two pumps so we had redundency but no one could advise us on a way to fully utilise them. i.e.
 * Alternate active pump to spread wear


Given we had 3 basement floods prior to finishing the basement this seemed like a poor solution.

The cause of the floodings were:
 * **Human Error** : Worker turned the power off!!
 * **Nature** : Lots of rain and GFCI tripoed.
 * **Mechanical Failure** : Float sensors jammed in pit and didn't activate.

So the 'only' solution was clearly to go to 11 on the problem. A control system that can:
 * Failover between the two power circuits
 * Monitor water depth
 * Monitor pump action
 * Alternate active pump
 * Fail safe... as in, if the control breaks then we have the same level of function as if the control system wasn't there.

This is still a WIP as the actual hardware has just been installed and monitoring is ongoing before full control is taken over by ths system.

## Goals

This 'box of tricks' is to serve anumber of purposes.

* Monitor the running of the pumps. So we can tell:
  * How frequently they operate.
  * Did the actually operate as expected.. has the pump failed and so needs replaced.
  * Montior the AC current usage to perhaps early detect failure or other operational issue.
  * Alternate the active pump. There are 2 in the pit and only using 1 will result in uneaven wear.
* There are two power circuites to the sump pit. If the RCD/GFCI trips for one circuit attempt to swap to use the other. 

## Still to come

### System design

Diagrams.. and diagrams :)

### Eagle Schematics for the hardware

### Home Automation configuration setup

### Actual Sump Pump controls
Configuration to set high and low water points once we measure what the current sensor reads for the manual floats


## Welcome to your Particle project!

Every new Particle project is composed of 3 important elements that you'll see have been created in your project directory for RelayControl.

#### ```/src``` folder:  
This is the source folder that contains the firmware files for your project. It should *not* be renamed. 
Anything that is in this folder when you compile your project will be sent to our compile service and compiled into a firmware binary for the Particle device that you have targeted.

If your application contains multiple files, they should all be included in the `src` folder. If your firmware depends on Particle libraries, those dependencies are specified in the `project.properties` file referenced below.

#### ```.ino``` file:
This file is the firmware that will run as the primary application on your Particle device. It contains a `setup()` and `loop()` function, and can be written in Wiring or C/C++. For more information about using the Particle firmware API to create firmware for your Particle device, refer to the [Firmware Reference](https://docs.particle.io/reference/firmware/) section of the Particle documentation.

#### ```project.properties``` file:  
This is the file that specifies the name and version number of the libraries that your project depends on. Dependencies are added automatically to your `project.properties` file when you add a library to a project using the `particle library add` command in the CLI or add a library in the Desktop IDE.

## Adding additional files to your project

#### Projects with multiple sources
If you would like add additional files to your application, they should be added to the `/src` folder. All files in the `/src` folder will be sent to the Particle Cloud to produce a compiled binary.

#### Projects with external libraries
If your project includes a library that has not been registered in the Particle libraries system, you should create a new folder named `/lib/<libraryname>/src` under `/<project dir>` and add the `.h`, `.cpp` & `library.properties` files for your library there. Read the [Firmware Libraries guide](https://docs.particle.io/guide/tools-and-features/libraries/) for more details on how to develop libraries. Note that all contents of the `/lib` folder and subfolders will also be sent to the Cloud for compilation.

## Compiling your project

When you're ready to compile your project, make sure you have the correct Particle device target selected and run `particle compile <platform>` in the CLI or click the Compile button in the Desktop IDE. The following files in your project folder will be sent to the compile service:

- Everything in the `/src` folder, including your `.ino` application file
- The `project.properties` file for your project
- Any libraries stored under `lib/<libraryname>/src`
