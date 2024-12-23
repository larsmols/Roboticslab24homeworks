input_file = "trajectory_data.txt"  # Replace with your input file path
output_file = "trajectory_data_processed.txt"  # Replace with your desired output file path

# Open the input file for reading
with open(input_file, "r") as infile:
    lines = infile.readlines()

# Open the output file for writing
with open(output_file, "w") as outfile:
    capture_position = False
    for line in lines:
        # Check if we are entering the position block
        if "position:" in line:
            capture_position = True
        elif "orientation:" in line:  # Exit the position block
            capture_position = False
        
        # If in position block, write the line to the output file
        if capture_position:
            outfile.write(line)
