import pandas as pd

# Read the extracted log data
df = pd.read_csv('path_to_your_extracted_log.csv')

# Extract the required columns
filtered_df = df[['utc', 'lat', 'lon', 'alt']]

# Convert the UTC time to the specified format
filtered_df['utc'] = pd.to_datetime(filtered_df['utc']).dt.strftime('%Y-%m-%dT%H:%M:%S.%f')

# Save the processed data to a new CSV file
filtered_df.to_csv('final_log.csv', index=False)
