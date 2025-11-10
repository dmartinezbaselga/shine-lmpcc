import sys
import matplotlib.pyplot as plt

from load_ros_data import *


def on_pick(event):
    # On the pick event, find the original line corresponding to the legend
    # proxy line, and toggle its visibility.
    legline = event.artist
    origline = lined[legline]
    visible = not origline.get_visible()
    origline.set_visible(visible)
    # Change the alpha on the line in the legend, so we can see what lines
    # have been toggled.
    legline.set_alpha(1.0 if visible else 0.2)
    fig.canvas.draw()


if __name__ == '__main__':
    simulation = sys.argv[1]
    planner_name = sys.argv[2]
    filename = simulation + "_" + planner_name

    print("----- plot_reference.py ------")
    data_path = os.getcwd() + "/../data/"  + filename + ".txt"
    data = load_ros_data(data_path)
    # print(type(data))

    # print("[Data]:")

    fig, ax = plt.subplots()

    lines = []
    lined = {}

    for key, value in data.items():
        # print("\t{}: {}".format(key, len(value)))
        t = range(len(value))
        line = ax.plot(t,value, 'o-', label=key)
        lines.extend(line)
        # ax.set_label(key)
        # print(type(key),key)
            
        

    leg = ax.legend()
    ax.grid()

    for legline, origline in zip(leg.get_lines(), lines):
        legline.set_picker(True)  # Enable picking on the legend line.
        lined[legline] = origline


    fig.canvas.mpl_connect('pick_event', on_pick)
    
    plt.show()