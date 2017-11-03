<p align="center">
  <h1 align="center"> ChampSim </h1>
</p>

# Compile

ChampSim takes five parameters: Branch predictor, L1D prefetcher, L2C prefetcher, LLC replacement policy, and number of cores. 
For example, `./build_champsim.sh bimodal no no lru 1` builds a single-core processor with bimodal branch predictor, no L1/L2 data prefetchers, and the baseline LRU replacement policy for the LLC.
```
$ ./build_champsim.sh bimodal no no lru 1

$ ./build_champsim.sh ${BRANCH} ${L1D_PREFETCHER} ${L2C_PREFETCHER} ${LLC_REPLACEMENT} ${NUM_CORE}
```

# Run simulation

Copy `scripts/run_champsim.sh` to the ChampSim root directory <br>

* Single-core simulation: Run simulation with `run_champsim.sh` script.

```
$ ./run_champsim.sh bimodal-no-no-lru-1core 1 10 bzip2

$ ./run_champsim.sh ${binary} ${n_warm} ${n_sim} ${trace} ${option}

${binary}: ChampSim binary compiled by "build_champsim.sh" (bimodal-no-no-lru-1core)
${n_warm}: number of instructions for warmup (1 million)
${n_sim}:  number of instructinos for detailed simulation (10 million)
${trace}: trace name (bzip2)
${option}: extra option for "-low_bandwidth" (src/main.cc)
```
Simulation results will be stored under "results_${n_sim}M" as a form of "${trace}-${binary}-${option}.txt".<br> 

* Multi-core simulation: Run simulation with `run_4core.sh` or `run_8core.sh`. <br>
Note that `${trace}` is replaced with `${num}` that represents a unique mixed workload ID for multi-programmed workloads. 

```
$ ./run_4core.sh ${binary} ${n_warm} ${n_sim} ${num} ${option}

${num}: mix number is the corresponding line number written in sim_list/4core_workloads.txt
```

# Add your own branch predictor, data prefetchers, and replacement policy
**Copy an empty template**
```
$ cp branch/branch_predictor.cc prefetcher/mybranch.bpred
$ cp prefetcher/l1d_prefetcher.cc prefetcher/mypref.l1d_pref
$ cp prefetcher/l2c_prefetcher.cc prefetcher/mypref.l2c_pref
$ cp replacement/llc_replacement.cc replacement/myrepl.llc_repl
```

**Work on your algorithms with your favorite text editor**
```
$ vim branch/mybranch.bpred
$ vim prefetcher/mypref.l1d_pref
$ vim prefetcher/mypref.l2c_pref
$ vim replacement/myrepl.llc_repl
```

**Compile and test**
```
$ ./build_champsim.sh mybranch mypref mypref myrepl 1
$ ./run_champsim.sh mybranch-mypref-mypref-myrepl-1core 1 10 bzip2
```


# How to create traces

We have included some sample traces, taken from SPEC CPU 2006 with SimPoint methodology.
The included Pin Tool champsim_tracer.so can be used to generate new traces.
It was created using Pin 3.0 (pin-3.0-76991-gcc-linux), and may require 
installing libdwarf.so, libelf.so, or other libraries, if you do not already 
have them.  Please refer to Pin documentation for working with Pin 3.0.

**Use the Pin tool like this**
```
pin -t lib/champsim_tracer.so -- <your program here>
```

The tracer has three options you can set:
```
-o
Specify the output file for your trace.
The default is default_trace.champsim

-s <number>
Specify the number of instructions to skip in the program before tracing begins.
The default value is 0.

-t <number>
The number of instructions to trace, after -s instructions have been skipped.
The default value is 1,000,000.
```
For example, you could trace 200,000 instructions of the program ls, after
skipping the first 100,000 instructions, with this command:
```
pin -t lib/champsim_tracer.so -o traces/ls_trace.champsim -s 100000 -t 200000 -- ls
```
Traces created with the champsim_tracer.so are 60 bytes per instruction,
but they generally compress down to 2-10 bytes per instruction using gzip.

# Evaluate Simulation

ChampSim measures the IPC (Instruction Per Cycle) value as a performance metric. <br>
There are also some useful shell scripts located under `scripts/` directory. <br>
