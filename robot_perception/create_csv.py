import pandas as pd

pose_of_tasks = [
    [0.4, 0.3, 0.3, 15, 20, 10],
    [2, 6, 10, 14, 18, 22],
    [3, 7, 11, 15, 19, 23],
    [4, 8, 12, 16, 20, 24]
]
columns = ["X", "Y", "Z", "ALPHA", "BETA", "GAMA"]
df = pd.DataFrame(pose_of_tasks, columns=columns)

df.to_csv("tasks.csv", index=False)

print("CSV file 'sample_rowwise_6x4.csv' has been created.")

"""import pandas as pd

# Read the CSV file
df = pd.read_csv("sample_rowwise_6x4.csv")

# Iterate through rows
for index, row in df.iterrows():
    print(f"Row {index}: {row.values}")"""