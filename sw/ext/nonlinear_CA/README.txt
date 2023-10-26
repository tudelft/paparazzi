This branch includes the to be tested changes made to the CA.

Added cost function term for derivative control action, not yet scaled to execution time of the optimizer.
Instead added additional timing restriction that limits execution time to 5mS.
Using Beta channel to communicate value for switch that informs about motor failure.
Adjusted position tracking gains according to pitching deck reference attitude test flight.


Max_iter 400 --> 75
Max_func_eval 2000 --> 75
Max time constraint --> 5000 uS. 
