#!/usr/bin/env gnuplot
# Gnuplot script to plot UAV attitude log data (Stick Mode only)
# Usage: gnuplot plot_uav_attitude_combined.gp
# Format: t roll_H roll_H_filtered pitch_H pitch_H_filtered (all in degrees)

# Input file
input_file = "uav_attitude.log"
output_file = "uav_attitude_combined.png"

# Set output format
set terminal pngcairo size 1920, 1080 enhanced font "Arial,12"
set output output_file

# Set multiplot layout: 2 rows, 1 column
set multiplot layout 2,1 title "UAV Attitude - Raw vs Filtered IMU" font "Arial,16"

# Common settings
set grid
set xlabel "Time (s)" font "Arial,11"
set datafile commentschars "#"  # Skip lines starting with #

# Plot 1: Roll - showing raw vs filtered
set title "Roll - Raw IMU (noisy) vs Filtered IMU (smooth)" font "Arial,12"
set ylabel "Roll (deg)" font "Arial,11"
plot input_file using 1:2 with lines title "roll_H (raw)" lw 1 lc rgb "red" dashtype 1, \
     input_file using 1:3 with lines title "roll_H_filtered" lw 2 lc rgb "blue" dashtype 1

# Plot 2: Pitch - showing raw vs filtered
set title "Pitch - Raw IMU (noisy) vs Filtered IMU (smooth)" font "Arial,12"
set ylabel "Pitch (deg)" font "Arial,11"
plot input_file using 1:4 with lines title "pitch_H (raw)" lw 1 lc rgb "red" dashtype 1, \
     input_file using 1:5 with lines title "pitch_H_filtered" lw 2 lc rgb "blue" dashtype 1

# Unset multiplot
unset multiplot

# Print completion message
print "Plot saved to: " . output_file

