import argparse

def main(argv=None):

    parser = argparse.ArgumentParser(description="publish a search mission.")
    parser.add_argument("--start-lat", type=float, default=47.500780)
    parser.add_argument("--start-lon", type=float, default=10.589547)
    parser.add_argument("--target-lat", type=float, default=47.49172040193652)
    parser.add_argument("--target-lon", type=float, default=10.58977333609236)
    args, unknown = parser.parse_known_args()

    with open("//wsl.localhost/.../need_help.txt", "w") as file:
        file.write(f"{args.start_lat};{args.start_lon};{args.target_lat};{args.target_lon}")

if __name__ == '__main__':
    main()
