# NOTES (TODO)
I dont know where to put this yet.
## Remarks on Nomenclature
- The robot arm (without any gripper) is refered to as the "arm" or "bioscara_arm".
- a custom bioscara gripper is refered to as the "gripper" or "bioscara_gripper_\<type\>"
- The assembly of arm and gripper is the "robot".

## Package Architecture
### Design goals
Modularity is key and scalability have been a key motivation when designing the package architecture. The architecture may serve as a blueprint for the future integration of more DALSA robots (custom or commercial) into ROS2. 

### Meta packages
<!-- The following needs a general explaination of packages in ROS2 -->
Source code for a a software module are grouped in packages, for example a hardware interface, custom controller or robot description. Modules that are neccessary for the operation of a hardware component, like an arm or gripper, are grouped in meta-packages. Meta-packages are simplify dependecy management. For example the dalsa_bioscara_arm meta-package specifies its dependencies on the packages its containing (the arms hardware interface, controllers, description and bringup packages). Other packages that depend on the bioscara_arm can simply specify the dalsa_bioscara_arm dependency and automatically inherit the remaining sub-depenencies.

<!-- TODO -->
#### Bringup
- Launch files to start the unit with the entire application stack which can run standalone
....
#### Description
- Contains a view launch to simply display the geometry visuals and collision bodies, test joint limits using joint state broadcaster and frame transformations.
#### Hardware Interface
#### Controllers
- meta package for dalsa_bioscara_arm, the robot arm itself


## Package Architecture
```mermaid
flowchart TB
 subgraph dalsa_bioscara_arm["dalsa_bioscara_arm"]
        bioscara_arm_bringup["bioscara_arm_bringup"]
        bioscara_arm_description["bioscara_arm_description"]
        bioscara_arm_hardware_interface["bioscara_arm_hardware_interface"]
        bioscara_arm_hardware_driver["bioscara_arm_hardware_driver"]
  end
 subgraph dalsa_grippers["dalsa_grippers"]
        bioscara_gripper_bringup["bioscara_gripper_bringup"]
        bioscara_gripper_description["bioscara_gripper_descriptions"]
        bioscara_gripper_hardware_driver["bioscara_gripper_hardware_driver"]
        bioscara_gripper_hardware_interface["bioscara_gripper_hardware_interface"]
  end
 subgraph dalsa_controllers["dalsa_controllers"]
        single_trigger_controller["single_trigger_controller"]
  end
    scene_description["scene_description"] --> bioscara_arm_description & bioscara_gripper_description
    scene_bringup["scene_bringup"] --> scene_description & dalsa_bioscara_arm & dalsa_grippers
    bioscara_arm_bringup --> single_trigger_controller & bioscara_arm_description
    bioscara_arm_description --> bioscara_arm_hardware_interface
    bioscara_arm_hardware_interface --> bioscara_arm_hardware_driver
    bioscara_gripper_bringup --> bioscara_arm_description & bioscara_gripper_description
    bioscara_gripper_hardware_driver --> bioscara_arm_hardware_driver
    bioscara_gripper_description --> bioscara_gripper_hardware_interface
    bioscara_gripper_hardware_interface --> bioscara_gripper_hardware_driver

```

singletirggercontroller as standalone package outside dalsa_biosacra:

is not tied and specific to bioscara



shared namespace between joints and gripper driver for simpler code

same for hardware interface

bioscara_gripper_descriptions i splural since it can host variants of the bioscara gipper. interface stays the same only description and parameters will change

<!-- TODO -->

## Scene Bringup
One-off launch file to launch the specific hardware combination, bioscara_arm and bioscara_gripper_128 in this case. 

would be nice if it could be a single launch file but. Each each setup is slightly different.