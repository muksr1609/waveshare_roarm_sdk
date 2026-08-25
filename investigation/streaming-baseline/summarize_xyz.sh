#!/bin/bash
# Summarize distinct XYZ values seen in the SDK's raw feedback prints
awk -F"'" '/T.. 1051/ {
  for(i=1;i<=NF;i++){
    if($i=="x"){x=$(i+1)}; if($i=="y"){y=$(i+1)}; if($i=="z"){z=$(i+1)}
  }
  gsub(/^[ \t]*:[ \t]*/,"",x); gsub(/^[ \t]*:[ \t]*/,"",y); gsub(/^[ \t]*:[ \t]*/,"",z)
  printf "%8.3f %8.3f %8.3f\n", x, y, z
}' /home/mukund/streaming_baseline/console.log | sort | uniq -c | sort -rn | head -15
