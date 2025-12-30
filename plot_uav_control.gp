#!/usr/bin/env gnuplot
# Plot control signals: target_roll, target_pitch, and pdpiRoll.Us
# Separated by mode (stick vs auto)

set terminal pngcairo size 1200,800 enhanced font "Arial,12"
set output "uav_control.png"

# Set multiplot layout: 3 rows, 1 column
set multiplot layout 3,1 title "Control Signals: target_roll, target_pitch, pdpiRoll.Us" font "Arial,14"

# Common settings
set grid
set xlabel "Time (s)"
set format x "%.1f"

# Plot 1: target_roll (both modes)
set title "Target Roll (Control Torque)"
set ylabel "Target Roll (torque)"
set yrange [-0.5:0.5]
plot "uav_control.log" using 1:($2==0 ? $3 : 1/0) with lines lw 2 lc rgb "blue" title "target_roll (stick)", \
     "uav_control.log" using 1:($2==1 ? $3 : 1/0) with lines lw 2 lc rgb "red" title "target_roll (auto)"

# Plot 2: target_pitch (both modes)
set title "Target Pitch (Control Torque)"
set ylabel "Target Pitch (torque)"
set yrange [-0.5:0.5]
plot "uav_control.log" using 1:($2==0 ? $4 : 1/0) with lines lw 2 lc rgb "blue" title "target_pitch (stick)", \
     "uav_control.log" using 1:($2==1 ? $4 : 1/0) with lines lw 2 lc rgb "red" title "target_pitch (auto)"

# Plot 3: pdpiRoll.Us (only valid in auto mode)
set title "pdpiRoll.Us (KrenCtrl Internal Signal)"
set ylabel "pdpiRoll.Us"
set yrange [*:*]
plot "uav_control.log" using 1:($2==1 ? $5 : 1/0) with lines lw 2 lc rgb "green" title "pdpiRoll.Us (auto mode only)"

unset multiplot

print "Control signals plot saved to: uav_control.png"

