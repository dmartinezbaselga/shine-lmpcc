SYSTEM="jackal"
cd ../lmpcc_solver/scripts
source venv/bin/activate
cd ${SYSTEM}
python3 ${SYSTEM}_solver.py $1 $2
deactivate
cd ../../../lmpcc
