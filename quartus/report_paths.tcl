# quartus_sta -t report_paths.tcl <revision>
set rev [lindex $quartus(args) 0]
project_open ltpi_timing -revision $rev
create_timing_netlist
read_sdc
update_timing_netlist
set out [open "paths_$rev.txt" w]
foreach_in_collection p [get_timing_paths -setup -npaths 5] {
    set from [get_node_info [get_path_info $p -from] -name]
    set to   [get_node_info [get_path_info $p -to] -name]
    puts $out "SLACK [format %.3f [get_path_info $p -slack]] : $from -> $to"
}
puts $out "FMAX_REPORT:"
report_clock_fmax_summary -file "fmax_$rev.txt"
close $out
project_close
