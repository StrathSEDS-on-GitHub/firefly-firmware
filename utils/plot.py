# videoseriesaligner: A tool to align a video with data feeds from the Firefly
# flight computer

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import os
import sys
from matplotlib import style
from moviepy import VideoFileClip
from argparse import ArgumentParser
import re
import itertools
from tqdm import tqdm

# Set up argument parser
parser = ArgumentParser(description="A tool to align a video with data feeds from the Firefly flight computer.")
required = parser.add_argument_group('required arguments')
required.add_argument("--csv", type=str, help="Path to the firefly log file.", required=True)
required.add_argument("--video", type=str, help="Path to the video file.", required=True)
required.add_argument("--data-align", type=pd.Timedelta, help="Timestamp to align in the data file.", required=True)
required.add_argument("--video-align", type=pd.Timedelta, help="Timestamp to align in the video", required=True)
required.add_argument("--sensor", type=str, help="Sensor type to be plotted", required=True)
required.add_argument("--csv-header", type=str, 
                    help="""Header of the csv file. Example: time,sensor,x0,x1,x2,y0..y2,z0..y2 """, required=True)
required.add_argument("--graph-title", type=str,
                    help="Title of the graph.", required=True)
required.add_argument("--y-title", type=str,
                    help="Title of the y-axis.", required=True)

optional = parser.add_argument_group('optional arguments')
optional.add_argument("--output", type=str, help="Path to output video file. Omit to plot to screen.", required=False)
optional.add_argument("--combinator", type=str,
                    help="Combinator to be used to combine multiple samples. \
                          Each sample name with the same index (i.e x0, y0, z0...) \
                          will be combined using the combinator.", required=False)
optional.add_argument("--sample-rate", type=int, help="Sample rate of the plotted sensor in Hz. Default: 100 Hz", default=100)
optional.add_argument("--assume-even-frame-timing", action='store_true', help="Assume the frame-timing is even. If this is off, ticks on the x-axis may have jumps in the animation", default=True)

args = parser.parse_args()
def format_millis(ms, pos):
    millis = int(ms % 1000)
    seconds = int((ms // 1000) % 60)
    minutes = int((ms // (1000 * 60)) % 60)
    hours = int((ms // (1000 * 60 * 60)) % 24)
    return f"{hours:02}:{minutes:02}:{seconds:02}.{millis:03}"

def generate_py_combinator(sample_names, cols, combinator):
    code =  "from math import *\ndef combinator(x):\n"
    for name,col in zip(sample_names, cols):
        code += f"    {name} = x['{col}']\n"

    code += f"    return {combinator}\n"
    globals_ = {}
    exec(code, globals_)
    return globals_['combinator']
        

def plot_data():
    if not os.path.exists(args.csv):
        print(f"File {args.csv} does not exist.")
        sys.exit(1)

    style.use('dark_background')

    header = args.csv_header.split(',')
    shorthand_regex = re.compile(r"^([a-zA-Z_]+)(\d+)\.\.\1(\d+)$")
    header = list(itertools.chain.from_iterable(
            [x] if shorthand_regex.search(x) is None
                else (
                    shorthand_regex.search(x).group(1) + str(i) 
                    for i in range(
                        int(shorthand_regex.search(x).group(2)), 
                        int(shorthand_regex.search(x).group(3)) + 1)
                )
            for x in header))

    df = pd.read_csv(args.csv, header=None, keep_default_na=True)
    if df.shape[1] != len(header):
        # drop extra columns
        df = df.iloc[:, :len(header)]
    df.columns = header

    print(df)

    df = df[df["sensor"] == args.sensor]
    if df.empty:
        print(f"No data found for sensor {args.sensor}.")
        sys.exit(1)

    df['time'] = pd.to_timedelta(df['time'])

    sample_headers = [x for x in header if x not in ['time', 'sensor']]
    sample_name_regex = re.compile(r"^([a-zA-Z_]+)(\d+)$")
    sample_names = list(dict.fromkeys(sample_name_regex.search(x).group(1) for x in sample_headers))
    sample_indices = list(dict.fromkeys(int(sample_name_regex.search(x).group(2)) for x in sample_headers))
    
    print("Sample names: ", sample_names)
    print("Indices: ", sample_indices)
    if sample_indices[-1] != (len(sample_indices) - 1):
        print(f"Invalid header: sample indices are not continuous.")
        sys.exit(1)

    # Combine samples
    if not args.combinator and len(sample_names) > 1:
        print(f"Invalid header: multiple named sample kinds found but no combinator specified.")
        sys.exit(1)

    sample_name = ""
    if args.combinator:
        for i in sample_indices:
            cols = [sample_name + str(i) for sample_name in sample_names]
            df.loc[:, f"combined_{i}"] = df.apply(generate_py_combinator(sample_names, cols, args.combinator), axis=1)
        sample_name = "combined_"
    else:
        sample_name = sample_names[0]
    

    series = [df[["time", sample_name + str(i)]].rename(columns={sample_name + str(i): "sample"}) for i in sample_indices]
    samples = pd.concat(series).sort_index(kind='merge')
    samples = samples.reset_index(drop=True)

    for i in samples.index[:-1]:
        if samples['time'].loc[i] >= samples['time'].loc[i + 1]:
            samples.loc[i + 1, 'time'] = samples['time'].loc[i] + pd.Timedelta(milliseconds=(1000//args.sample_rate))
    samples['times'] = samples['time'].apply(lambda x: format_millis(x.total_seconds() * 1000, None))

    print("times ms:\n", samples['time'] / pd.Timedelta(microseconds=1000))

    frame_time_ms = ((samples['time'].iloc[-1] - samples['time'].iloc[0])) / pd.Timedelta(microseconds=1000) / len(samples)
    print(f"Average frame time: {frame_time_ms}")

    # Align the data with the video
    clip = VideoFileClip(args.video, target_resolution=(1280, 720))

    if clip.reader is None:
        print("Error: could not read video file.")
        sys.exit(1)

    align_sample_idx = samples[samples["time"] >= args.data_align].index[0]
    framecount = clip.reader.n_frames
    video_duration_frames = clip.duration * 1000 / frame_time_ms
    video_align_frame: int = (args.video_align) / pd.Timedelta(milliseconds=frame_time_ms)
    print("video align frame", video_align_frame)

    samples_start = max(int(align_sample_idx - video_align_frame), 0)
    samples_end = min(int(samples_start + video_duration_frames), len(samples))

    video_frames_per_sample = (frame_time_ms * clip.fps / 1000)
    print("Video frames per sample: ", video_frames_per_sample)

    samples_before_align = align_sample_idx - samples_start
    samples_after_align = samples_end - align_sample_idx

    print("samples before align: ", samples_before_align)
    print("samples after align: ", samples_after_align)

    print((video_align_frame + samples_after_align) , framecount);


    video_start_frame = max(int((video_align_frame - samples_before_align) * video_frames_per_sample), 0)
    video_end_frame = min(int((video_align_frame + samples_after_align) * video_frames_per_sample), framecount)


    print("Video range: ", video_start_frame, video_end_frame)
    samples = samples.iloc[samples_start:samples_end]
    samples = samples.reset_index(drop=True)
    print("Sample time: ", samples['time'].iloc[-1] - samples['time'].iloc[0])

    # Create a figure and axis

    font = {'fontname':'Libre Baskerville'}
    fig, axs = plt.subplots(2, gridspec_kw={'height_ratios': [1, 2]})
    fig.tight_layout(rect=(0, 0, 1, 0.95), h_pad=0.5)
    fig.suptitle(args.graph_title, fontsize=18,  **font)
    fig.set_size_inches(12, 8)
    g,v = axs

    # Set the title and labels
    g.set_xlabel("Time", **font)
    g.set_ylabel(args.y_title, **font)

    # Set the x and y limits
    miny, maxy = samples['sample'].min(), samples['sample'].max()
    spany = maxy - miny
    miny, maxy = miny - spany * 0.1, maxy + spany * 0.1

    g.set_ylim(miny, maxy)

    # Load video file imu.mov and show it in the second subplot
    # alongside the graph
    v.axis('off')

    samples["time"] /= pd.Timedelta(milliseconds=1)

    print("Samples to show:\n", samples)

    plot_col = "times" if args.assume_even_frame_timing else "time"

    pbar = tqdm(total=len(samples), desc="Plotting", unit="frames")
    # Plot the data
    def animate(i):
        pbar.update(1)
        if i > 0:
            g.cla()
            j = i
            g.set_xlabel("Time", **font)
            g.set_ylabel(args.y_title, **font)
            g.plot(samples[plot_col][0:j], samples['sample'][0:j])
            # show first and last x ticks
            xticks = [samples[plot_col].iloc[0], samples[plot_col].iloc[j]]
            g.set_xticks(xticks)
            # set formatters

            g.set_ylim(miny, maxy)

            # show grid lines
            g.grid(axis='y', color='gray', linestyle='--', linewidth=0.5)

            v.cla()
            v.axis('off')
            pct_done = (video_start_frame + i * video_frames_per_sample) / framecount
            frame = clip.get_frame(pct_done * clip.duration)
            # rotate the frame by 90 degrees
            # frame = np.rot90(frame)
            v.imshow(frame)

    # Create the animation
    ani = animation.FuncAnimation(fig, animate,
                                  frames=(len(samples)), interval=frame_time_ms, blit=False)
    if args.output:
        ani.save(args.output, writer='ffmpeg', fps=1000/frame_time_ms, dpi=100)
    else:
        plt.show()


plot_data()
