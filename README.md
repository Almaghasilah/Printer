# Printer
Designing a printer through MATLAB for Feedback Class (ECE414)

# Specification of the Design 
The ink cartidge starts 2cm before the edge of the paper. It must travel this 2 cm in no more than 160 ms. Once it reaches the edge of the
paper, it must travel at 66cm/s with +/-0.01% velocity error tolerance. After traveling 22 cm, the ink cartidge has 2 cm to slow down and stop in no more than 160 ms before returning across the page in the opposite direction. The ink cartidge cannot go backward from its initial position of zero, nor past the end point 2 cm beyond the far edge of the paper. It also cannot strike either of these limits at velocity greater than 0.25cm/s. Your analysis need only consider one forward pass paper since the return trip will have the same response. 






