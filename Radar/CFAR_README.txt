This README is a brief description of the 2D CFAR algorithm implemented in radar_target_generation_and_detection.m

Implementation Steps

 1. Select the number of training and guard cells around the cell under test (CUT)
     - Use separate variables/values for each dimension (range and dopplar)
	 - I used the default values suggested in the project instructions
	 - The default values worked well
 2. Select the SNR offset to add to the noise threshhold to reduce false positives
     - I started with the default value suggested in the project instructions
     - I had to increase it to deal with false positives
 3. Loop over all cells that are eligible to be the CUT
     - Cells near the edge are not eligible since they do not have the right number of guard and training cells available
 4. For each CUT, use the training cells to calculate the noise threshhold
     - Exclude the guard cells from the calculation
 5. Reset the CUT based on whether it is a valid target
	 - Set to zero if the previous CUT value is less than the noise threshold plus the SNR offset
	 - Otherwise set to one
 6. Set the cells at the edge that were not valid to be the CUT to zero
