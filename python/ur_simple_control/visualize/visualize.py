import numpy as np
import matplotlib.pyplot as plt
from collections import deque, namedtuple
import time
import copy
from pinocchio.visualize import MeshcatVisualizer

# tkinter stuff
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from tkinter import *
from tkinter.ttk import *

# rows and cols are flipped lel
def getNRowsMColumnsFromTotalNumber(n_plots):
    if n_plots == 1:
        n_cols = 1
        n_rows = 1
    if n_plots == 2:
        n_rows = 2
        n_cols = 1
    # i'm not going to bother with differently sized plots 
    if (n_plots == 3) or (n_plots == 4):
        n_cols = 2
        n_rows = 2
    if (n_plots == 5) or (n_plots == 6):
        n_cols = 2
        n_rows = 3
    if (n_plots >= 7) and (n_plots <= 9):
        n_cols = 3
        n_rows = 3
    if n_plots >= 10:
        raise NotImplementedError("sorry, you can only do up to 9 plots. more require tabs, and that's future work")
    return n_rows, n_cols


""" 
plotFromDict
------------
plots logs stored in a dictionary
- every key is one subplot, and it's value
  is what you plot
""" 
def plotFromDict(plot_data, final_iteration, args):
    # TODO: replace with actual time ( you know what dt is from args)
    t = np.arange(final_iteration)
    n_cols, n_rows = getNRowsMColumnsFromTotalNumber(len(plot_data))
    # this is what subplot wants
    subplot_col_row = str(n_cols) + str(n_rows)
    ax_dict ={}
    # NOTE: cutting off after final iterations is a vestige from a time
    # when logs were prealocated arrays, but it ain't hurtin' nobody as it is
    for i, data_key in enumerate(plot_data):
        colors = plt.cm.jet(np.linspace(0, 1, plot_data[data_key].shape[1]))
        ax_dict[data_key] = plt.subplot(int(subplot_col_row + str(i + 1)))
        for j in range(plot_data[data_key].shape[1]):
            ax_dict[data_key].plot(t, plot_data[data_key][:final_iteration,j], color=colors[j], label=data_key + "_" + str(j))
        ax_dict[data_key].legend()
    plt.show()


"""
realTimePlotter
---------------
- true to its name
"""
# STUPID MATPLOTLIB CAN'T HANDLE MULTIPLE FIGURES FROM DIFFERENT PROCESS
# LITERALLY REFUSES TO GET ME A FIGURE
def realTimePlotter(args, queue):
    if args.debug_prints:
#        print("REAL_TIME_PLOTTER: real time visualizer has been summoned")
        print("REAL_TIME_PLOTTER: i got this queue:", queue)
    plt.ion()
    fig = plt.figure()
    canvas = fig.canvas
    queue.put("success")
    logs_deque = {}
    logs_ndarrays = {}
    AxisAndArtists = namedtuple("AxAndArtists", "ax artists")
    axes_and_updating_artists = {}
    if args.debug_prints:
        print("REAL_TIME_PLOTTER: i am waiting for the first log_item to initialize myself")
    log_item = queue.get()
    if len(log_item) == 0:
        print("you've send me nothing, so no real-time plotting for you")
        return
    if args.debug_prints:
        print("REAL_TIME_PLOTTER: got log_item, i am initializing the desired plots")
    ROLLING_BUFFER_SIZE = 100
    t = np.arange(ROLLING_BUFFER_SIZE)

    n_cols, n_rows = getNRowsMColumnsFromTotalNumber(len(log_item))
    # this is what subplot wants
    subplot_col_row = str(n_cols) + str(n_rows)
    # preload some zeros and initialize plots
    for i, data_key in enumerate(log_item):
        # you give single-vector numpy arrays, i instantiate and plot lists of these 
        # so for your (6,) vector, i plot (N, 6) ndarrays resulting in 6 lines of length N.
        # i manage N because plot data =/= all data for efficiency reasons.
        assert type(log_item[data_key]) == np.ndarray
        assert len(log_item[data_key].shape) == 1
        # prepopulate with zeros via list comperhension (1 item is the array, the queue is 
        # ROLLING_BUFFER_SIZE of such arrays, and these go in and out at every time step)
        logs_deque[data_key] = deque([log_item[data_key] for index in range(ROLLING_BUFFER_SIZE)])
        # i can only plot np_arrays, so these queues will have to be turned to nparray at every timestep
        # thankfull, deque is an iterable
        logs_ndarrays[data_key] = np.array(logs_deque[data_key])
        colors = plt.cm.jet(np.linspace(0, 1, log_item[data_key].shape[0]))
        ax = fig.add_subplot(int(subplot_col_row + str(i + 1)))
        # some hacks, i'll have to standardize things somehow
        if data_key == 'qs':
            ax.set_ylim(bottom=-6.14, top=6.14)
        if data_key == 'dmp_poss':
            ax.set_ylim(bottom=-6.14, top=6.14)
        if data_key == 'dqs':
            ax.set_ylim(bottom=-1.7, top=1.7)
        if data_key == 'dmp_vels':
            ax.set_ylim(bottom=-1.7, top=1.7)
        if 'wrench' in data_key:
            ax.set_ylim(bottom=-20.0, top=20.0)
        if data_key == 'tau':
            ax.set_ylim(bottom=-2.0, top=2.0)
        axes_and_updating_artists[data_key] = AxisAndArtists(ax, {})
        for j in range(log_item[data_key].shape[0]):
            # the comma is because plot retuns ax, sth_unimportant.
            # and python let's us assign iterable return values like this
            axes_and_updating_artists[data_key].artists[str(data_key) + str(j)], = \
                    axes_and_updating_artists[data_key].ax.plot(t, logs_ndarrays[data_key][:,j], 
                                                             color=colors[j], label=data_key + "_" + str(j))
        axes_and_updating_artists[data_key].ax.legend(loc='upper left')

    # need to call it once
    canvas.draw()
    canvas.flush_events()
    background = fig.bbox

    if args.debug_prints:
        print("REAL_TIME_PLOTTER: FULLY ONLINE")
    try:
        while True:
            log_item = queue.get()
            if log_item == "befree":
                if args.debug_prints:
                    print("REAL_TIME_PLOTTER: got befree, realTimePlotter out")
                break
            for data_key in log_item:
                # remove oldest
                logs_deque[data_key].popleft()
                # put in new one
                logs_deque[data_key].append(log_item[data_key])
                # make it an ndarray (plottable)
                logs_ndarrays[data_key] = np.array(logs_deque[data_key])
                # now shape == (ROLLING_BUFFER_SIZE, vector_dimension)
                for j in range(logs_ndarrays[data_key].shape[1]):
                    axes_and_updating_artists[data_key].artists[str(data_key) + str(j)].set_data(t, logs_ndarrays[data_key][:,j])
                    axes_and_updating_artists[data_key].ax.draw_artist(\
                            axes_and_updating_artists[data_key].artists[str(data_key) + str(j)])
            canvas.blit(fig.bbox)
            canvas.flush_events()
    except KeyboardInterrupt:
        if args.debug_prints:
            print("REAL_TIME_PLOTTER: caught KeyboardInterrupt, i'm out")
    plt.close(fig)


def manipulatorVisualizer(args, model, collision_model, visual_model, queue):
    # for whatever reason the hand-e files don't have/
    # meshcat can't read scaling information.
    # so we scale manually
    for geom in visual_model.geometryObjects:
        if "hand" in geom.name:
            s = geom.meshScale
            # this looks exactly correct lmao
            s *= 0.001
            geom.meshScale = s
    for geom in collision_model.geometryObjects:
        if "hand" in geom.name:
            s = geom.meshScale
            # this looks exactly correct lmao
            s *= 0.001
            geom.meshScale = s
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()
    print("MANIPULATORVISUALIZER: FULLY ONLINE")
    try:
        while True:
            q = queue.get()
            if type(q) == str:
                print("got str q")
                if q == "befree":
                    if args.debug_prints:
                        print("MANIPULATORVISUALIZER: got befree, manipulatorVisualizer out")
                    # TODO: find a way to actually close it, i don't want a bilion dangling sockets
                    viz.viewer.window.server_proc.kill()
                    viz.viewer.window.server_proc.wait()
                    break
            viz.display(q)
    except KeyboardInterrupt:
        if args.debug_prints:
            print("MANIPULATORVISUALIZER: caught KeyboardInterrupt, i'm out")
        # TODO: find a way to actually close it, i don't want a bilion dangling sockets
        # and a random explosion caused by them
        #viz.viewer.close()
        viz.viewer.window.server_proc.kill()
        viz.viewer.window.server_proc.wait()
