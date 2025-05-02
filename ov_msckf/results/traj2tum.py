import pandas as pd
import argparse
import os

def traj2tum(traj_file, tum_file):
    df = pd.read_csv(traj_file, sep=" ")
    print(df)
    tum_df = df.iloc[:, :8]
    tum_df.columns=["#timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]
    tum_df.to_csv(tum_file, index=False, sep=' ')


def main():
    parser = argparse.ArgumentParser(description="Process traj and tum file paths.")
    parser.add_argument("--traj", required=True, help="Path to the traj file.")
    parser.add_argument("--tum", help="Path to the output tum file.")
    
    args = parser.parse_args()
    
    traj_path = args.traj
    tum_path = args.tum

    # 如果没有提供 tum 参数，则使用 traj 文件所在目录并更改后缀为 .tum
    if not tum_path:
        base_dir = os.path.dirname(traj_path)
        base_name = os.path.splitext(os.path.basename(traj_path))[0]
        tum_path = os.path.join(base_dir, f"{base_name}.tum")
    
    print(f"Traj file: {traj_path}")
    traj2tum(traj_path, tum_path)
    print(f"Tum file: {tum_path}")

if __name__ == "__main__":
    main()
