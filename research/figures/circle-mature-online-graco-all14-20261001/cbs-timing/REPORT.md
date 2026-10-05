# CBS computation and graph growth

The local solver remained inexpensive as the published global graph grew from 72 to 366 submap poses and from 22 to 110 admitted GPU registration pairs. These are 14 concurrent local optimizers, not one solver containing the whole global graph. The largest recorded local estimate has 61 variables.

Across 842 recorded nonterminal updates, solver wall time was 0.351 ms median, 1.003 ms at the 95th percentile and 4.499 ms maximum. Full timer callbacks were 1.629 / 32.230 / 1188.685 ms respectively. The callback includes lock waiting, successful input preparation/admission, registration preparation, the solver update, residual/statistics evaluation, asynchronous request submission and visualization. A callback returning before performing an update is not in this CSV.

The 3,930 logged peer-service responses took 0.379 ms median, 2.385 ms at the 95th percentile and 190.248 ms maximum. These include request-lock waiting, cavity marginalization and response packing. They are separate from the update callback and are not summed into a fictitious synchronized global cycle.

All times are elapsed wall time, not CPU time or a GPU kernel microbenchmark. GPU work can be charged to different enclosing calls; the full callback and peer-service measurements are the broader scopes. Waiting for asynchronous peer replies, DDS delivery, separate PCM/exchange callbacks and Python bridge preparation are not a complete end-to-end latency measurement here.

Update timestamps come from the matching robot/iteration statistics receipt. Peer-service CSV rows contain only an iteration number, so their plotted times are assigned to the serving robot’s iteration timestamp, approximately locating each request within that iteration. Requests before iteration 1 have no matching timestamp and appear only in the retained data and duration statistics. Duration measurements are native steady-clock measurements. One terminal timer callback per robot is missing by design and is never filled with zero; the final geometry-validation cost is therefore not covered by these update maxima.

The actual recorded configuration selects outgoing pose or anchor exchange randomly at each iteration. It does not strictly alternate. Both categories run the local optimizer; the labels describe the exchange selected after the update, not two independent solver kernels. Earlier report wording has been corrected.

Gray shading marks time after the last bag ended. The late decrease in cost occurs while the graph is fixed and fewer new beliefs/factors arrive; it does not imply that larger graphs are intrinsically cheaper. The recorded solver updates show no growth-related bottleneck at this tested size, but this is not a controlled asymptotic scalability experiment or a Jetson benchmark.
