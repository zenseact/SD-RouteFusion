import h5py
import numbers

def save_recursively(hdf_group: h5py.Group, data: dict):
    """Saves a whole dictionary into and hdf5 group."""
    for key, value in data.items():
        if isinstance(value, dict):
            group = hdf_group.create_group(key)
            save_recursively(group, value)
        if isinstance(value, str):
            hdf_group.attrs.create(key, value)
        if isinstance(value, list) or isinstance(value, numbers.Number):
            hdf_group.create_dataset(key, data=value)


def load_recursively(hdf_obj, keys_to_extract: list = []):
    data = {}
    # Extract attributes
    for name, attr_data in hdf_obj.attrs.items():
        if not keys_to_extract or name in keys_to_extract:
            data[name] = attr_data
    for key in hdf_obj.keys():
        if isinstance(hdf_obj[key], h5py.Group):
            # Extract everything in the group
            data[key] = load_recursively(hdf_obj[key], keys_to_extract)
            if not data[key]:
                data.pop(key)
        if isinstance(hdf_obj[key], h5py.Dataset):
            if not keys_to_extract or key in keys_to_extract:
                data[key] = hdf_obj[key][()]

    return data


class DatasetFile:
    def __init__(self, filename: str, write: bool = False):
        self.filename_ = filename
        if write:
            # Creates an empty file
            with h5py.File(self.filename_, "w") as f:
                pass

    def add_sample(self, sample_id: str, data: dict):
        # Append to the existing file
        with h5py.File(self.filename_, "a") as f:
            f.create_group(sample_id)
            save_recursively(f[sample_id], data)

    def load_sample(self, sample_id: str, keys_to_extract = []):
        with h5py.File(self.filename_, "r") as f:
            data = {}
            if sample_id in f.keys():
                data[sample_id] = load_recursively(f[sample_id], keys_to_extract)
            else:
                print(f"Could not load {sample_id}. Not found in {self.filename_}.")

            return data
    
    def get_file_information(self):
        information = {}
        with h5py.File(self.filename_, "r") as f:
            information["groups"] = list(f.keys())
            information["attributes"] = dict(f.attrs)

        return information

    def load_all(self, keys_to_extract: list = []):
        data = {}
        info = self.get_file_information()
        with h5py.File(self.filename_, "r") as f:
            for sample_id in info["groups"]:
                if sample_id in f.keys():
                    data[sample_id] = load_recursively(f[sample_id], keys_to_extract)
                else:
                    print(f"Could not load {sample_id}. Not found in {self.filename_}.")

            return data


def filter_by_gt(filename_in: str, filename_out: str, keys_to_extract: list = []):
    df_in = DatasetFile(filename_in)
    df_out = DatasetFile(filename_out, write=True)
    info = df_in.get_file_information()
    for sample_id in info["groups"]:
        sample = df_in.load_sample(
                sample_id,
                keys_to_extract,
        )
        if "ground_truth_data" in sample.keys():
            df_out.add_suite(sample)


if __name__ == "__main__":

    keys_to_extract = ["frame_timestamp_date", "holistic_path_ground_truth_rear_axle_coordinates"]
    # Load only the first sample
    df = DatasetFile("/mnt/regtt01/mre/msc_thesis/vision_10/json/full_dataset.hdf5")
    info = df.get_file_information()
    single_sample = df.load_sample(info["groups"][0], keys_to_extract)

    print(f"Single sample extraction with keys:", keys_to_extract)
    print(single_sample)

    # Load everything
    whole_dataset = df.load_all()
    print("Whole dataset:", whole_dataset)


    # Filtering by GT
    filter_by_gt("/mnt/regtt01/mre/msc_thesis/vision_10/json/full_dataset.hdf5",
                 "/mnt/regtt01/mre/msc_thesis/vision_10/json/filtered_dataset.hdf5")
