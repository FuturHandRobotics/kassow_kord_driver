For each Kassow KR810 robot arm (identified by the last two digits of the serial no, found on arm base), run [this patching script (futur_arm_planning/scripts/patch_xacro_from_urdf.py)](https://github.com/FuturHandRobotics/futur_arm_control/commit/c0cba17d66144791941b6b1f49432264b08e57a0#diff-ca354cde8497fd260d3adf062defb91a81f6c53be478e6d5adf82eb689eec56c) to get the kinematically correct xacro file.

Eg.
```
cd ~/FuturHub/src/futur_code/futur_arm_control/futur_arm_planning/scripts/
```
```
python3 patch_xacro_from_urdf.py \
--urdf ~/FuturHub/src/futur_code/futur_arm_control/futur_arm_planning/urdf/model_42.urdf \
--xacro ~/FuturHub/external/kassow-kord-driver/kassow_kord_description/kr810/urdf/kr810_description_nominal.urdf.xacro \
--out ~/FuturHub/external/kassow-kord-driver/kassow_kord_description/kr810/urdf/kr810_description_patched_42.urdf.xacro
```
