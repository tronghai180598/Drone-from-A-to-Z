#!/usr/bin/env gnuplot
# Gnuplot script to plot UAV attitude log data with both modes on same plot
# Usage: gnuplot plot_uav_attitude_combined.gp
# Format: t mode roll_set roll pitch_set pitch
# Mode: 0 = stick mode, 1 = auto mode

# Input file
input_file = "uav_attitude.log"
output_file = "uav_attitude_combined.png"

# Set output format
set terminal pngcairo size 1920, 1080 enhanced font "Arial,12"
set output output_file

# Set multiplot layout: 2 rows, 1 column
set multiplot layout 2,1 title "UAV Attitude Control Analysis - Combined Modes" font "Arial,16"

# Common settings
set grid
set xlabel "Time (s)" font "Arial,11"
set datafile commentschars "#"  # Skip lines starting with #

# Plot 1: Roll control (both modes)
set title "Roll Control - Stick Mode (red/blue) vs Auto Mode (orange/cyan)" font "Arial,12"
set ylabel "Roll (rad)" font "Arial,11"
plot input_file using 1:($2==0 ? $3 : NaN) with lines title "roll_set (stick)" lw 2 lc rgb "red" dashtype 1, \
     input_file using 1:($2==0 ? $4 : NaN) with lines title "roll (stick)" lw 2 lc rgb "blue" dashtype 1, \
     input_file using 1:($2==1 ? $3 : NaN) with lines title "roll_set (auto)" lw 2 lc rgb "orange" dashtype 2, \
     input_file using 1:($2==1 ? $4 : NaN) with lines title "roll (auto)" lw 2 lc rgb "cyan" dashtype 2

# Plot 2: Pitch control (both modes)
set title "Pitch Control - Stick Mode (red/blue) vs Auto Mode (orange/cyan)" font "Arial,12"
set ylabel "Pitch (rad)" font "Arial,11"
plot input_file using 1:($2==0 ? $5 : NaN) with lines title "pitch_set (stick)" lw 2 lc rgb "red" dashtype 1, \
     input_file using 1:($2==0 ? $6 : NaN) with lines title "pitch (stick)" lw 2 lc rgb "blue" dashtype 1, \
     input_file using 1:($2==1 ? $5 : NaN) with lines title "pitch_set (auto)" lw 2 lc rgb "orange" dashtype 2, \
     input_file using 1:($2==1 ? $6 : NaN) with lines title "pitch (auto)" lw 2 lc rgb "cyan" dashtype 2

# Unset multiplot
unset multiplot

# Print completion message
print "Plot saved to: " . output_file

