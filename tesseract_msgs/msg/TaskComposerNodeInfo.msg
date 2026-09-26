# This message corresponds to tesseract::task_composer::TaskComposerNodeInfo

# Name of the Task
string name

# Unique ID generated for the Task by Taskflow
string uuid

# Inbound Edges
string[] inbound_edges

# Outbound Edges
string[] outbound_edges

# Input port mappings
TaskComposerPortMapping[] input_port_mappings

# Output port mappings
TaskComposerPortMapping[] output_port_mappings

# Value returned from the Task on completion
int32 return_value

# Status code
int32 status_code

# Status message
string status_message

# Time elapsed
float64 elapsed_time
