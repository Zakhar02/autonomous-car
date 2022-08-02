for f in data/*.lcm; do
    lcm-gen -p ./"$f"
done
export PYTHONPATH='.'
python3.7 /home/zakhar/work/SI/plant/plot.py &
sleep 0.3
python3.7 /home/zakhar/work/SI/plant/plant.py &
python3.7 /home/zakhar/work/SI/planning/planning.py &
sleep 0.1
python3.7 /home/zakhar/work/SI/control/control.py &
sleep 0.5
python3.7 /home/zakhar/work/SI/init.py
