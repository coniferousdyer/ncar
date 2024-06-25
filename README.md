# nCAR
This repository contains an implementation of the heuristic <b>nearest neighbour-Based Clustering and Routing (nCAR)</b> algorithm, proposed in the paper "A Scalable Multi-Robot Task Allocation Algorithm" by Chayan Sarkar, Himadri Sekhar Paul and Arindam Pal of TCS Research and Innovation, Kolkata. The paper can be found [here](https://sci-hub.se/10.1109/icra.2018.8460886). The algorithm provides an approximate solution to the NP-hard Capacity-Constrained Vehicle Routing Problem (CVRP), and performs better than the state-of-the-art heuristics, while giving a solution comparable to the optimal one, even for a very high order. The state-of-the-art solution, Google’s OR-Tools has 1.5 times the execution time and number of routes of nCAR, and nCAR has a runtime speedup of 6 when the item size is 2000.

## Instructions

First, you'll need to create a text file `input.txt` in the same directory as the code files. A sample has been included in the repository. Assuming there are n objects, the input format is as follows:

- <b>First line:</b> Number of objects 
- <b>Second line:</b> Capacity of each robot
- <b>Third line:</b> Weight of each object (n integers)
- <b>Next n + 1 lines:</b> Distance between each pair of objects (n + 1 lines, each line with n + 1 integers)

To run the code, run `ncar.py`. For instance, you can use the following command:

```console
python3 ncar.py
```

## Results

Detailed information about the results and the algorithm implementation itself can be found in `report.pdf`.
