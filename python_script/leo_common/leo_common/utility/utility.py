def load_from_file(file_path):
    file = open(file_path)
    # read all data
    file_data = file.read()
    # split file by lines
    lines = file_data.replace(",", " ").replace("\t", " ").split("\n")
    # split each line
    data_list = [[v.strip() for v in line.split(" ") if v.strip() != ""]
                 for line in lines if len(line) > 0 and line[0] != "#"]
    return data_list
