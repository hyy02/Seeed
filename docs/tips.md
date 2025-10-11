## 1.PID adapt for the xlerobot(final version)
```python
    def configure(self):
        # Set-up arm actuators (position mode)
        # We assume that at connection time, arm is in a rest position,
        # and torque can be safely disabled to run calibration        
        self.bus1.disable_torque()
        self.bus2.disable_torque()
        self.bus2.configure_motors()
        self.bus2.configure_motors()
        
        for name in self.left_arm_motors:
            if name in ["left_arm_shoulder_lift", "left_arm_elbow_flex"]:
                # These two motors have more load, set to higher P_Coefficient
                self.bus1.write("Operating_Mode", name, OperatingMode.POSITION.value)
                self.bus1.write("P_Coefficient", name, 6)
                self.bus1.write("I_Coefficient", name, 0)
                self.bus1.write("D_Coefficient", name, 8)
                self.bus1.write(
                "Max_Torque_Limit", name, 500
                )  # 50% of the max torque limit to avoid burnout
                continue

            self.bus1.write("Operating_Mode", name, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.bus1.write("P_Coefficient", name, 4)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.bus1.write("I_Coefficient", name, 0)
            self.bus1.write("D_Coefficient", name, 8)
            self.bus1.write(
                "Max_Torque_Limit", name, 500
            )  # 50% of the max torque limit to avoid burnout

        
        for name in self.head_motors:
            self.bus1.write("Operating_Mode", name, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.bus1.write("P_Coefficient", name, 4)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.bus1.write("I_Coefficient", name, 0)
            self.bus1.write("D_Coefficient", name, 8)
            self.bus1.write(
                "Max_Torque_Limit", name, 500
            )  # 50% of the max torque limit to avoid burnout
        
        for name in self.right_arm_motors:
            if name in ["right_arm_shoulder_lift", "right_arm_elbow_flex"]:
                # These two motors have more load, set to higher P_Coefficient
                self.bus2.write("Operating_Mode", name, OperatingMode.POSITION.value)
                self.bus2.write("P_Coefficient", name, 6)
                self.bus2.write("I_Coefficient", name, 0)
                self.bus2.write("D_Coefficient", name, 8)
                self.bus2.write(
                "Max_Torque_Limit", name, 500
            )  # 50% of the max torque limit to avoid burnout

                continue
            
            self.bus2.write("Operating_Mode", name, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.bus2.write("P_Coefficient", name, 4)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.bus2.write("I_Coefficient", name, 0)
            self.bus2.write("D_Coefficient", name, 8)
            self.bus2.write(
                "Max_Torque_Limit", name, 500
            )  # 50% of the max torque limit to avoid burnout
```
        
        for name in self.base_motors:
            self.bus2.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        
        
        self.bus1.enable_torque()
        self.bus2.enable_torque()
