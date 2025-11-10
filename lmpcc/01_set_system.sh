if [ $1 == "Carla" ]
then
    system="Prius"

elif [ $1 == "SimpleSim" ]
then
    system="Prius"

elif [ $1 == "JackalSimulator" ]
then
    system="Jackal"

elif [ $1 == "JackalSocnavbench" ]
then
    system="Jackal"

elif [ $1 == "HovergamesGazebo" ]
  then
    system="Hovergames"

else
	system=$1
fi

sed -i '/set(CONFIGURATION_TO_USE/ c\set(CONFIGURATION_TO_USE "'$1'")' CMakeLists.txt
sed -i '/set(SYSTEM_TO_USE/ c\set(SYSTEM_TO_USE "'${system}'")' CMakeLists.txt
sed -i '/set(SYSTEM_TO_USE/ c\set(SYSTEM_TO_USE "'${system}'")' ../lmpcc_solver/CMakeLists.txt
if test -f "../../../.vscode/c_cpp_properties.json"; then
	sed -i '/SYSTEM_/ c\\t\t\t\t"SYSTEM_'$1'\"' ../../../.vscode/c_cpp_properties.json
fi

echo 'Configuration set to:' $1
echo 'System set to:' ${system}

sed -i '/SYSTEM=/ c\SYSTEM="'${system,,}'"' 02_generate_solver.sh