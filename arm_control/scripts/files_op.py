import os
import shutil
import yaml

def read_yaml(yaml_path):
    with open(yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    return data

def write_yaml(yaml_path, data):
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.dump(data, f)

def clear_folder(folder_path):
    # 检查文件夹是否存在
    if os.path.exists(folder_path) and os.path.isdir(folder_path):
        # 遍历文件夹内的所有文件和子文件夹
        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)
            try:
                # 如果是文件，则删除
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                # 如果是目录，则删除整个目录
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                print(f'删除 {file_path} 时出错: {e}')
    else:
        print(f'文件夹 {folder_path} 不存在或不是一个目录')