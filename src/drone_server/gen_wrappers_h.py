import glob, os
print("Generating drone wrapper header file")
wrappers_dir_rel = "../../wrappers"
wrappers_dir = os.path.dirname(os.path.realpath(__file__)) + "/" + wrappers_dir_rel

wrapper_h_path = os.path.dirname(os.path.realpath(__file__))
wrapper_h = open(wrapper_h_path + "/wrappers.h", 'wt')
wrapper_h.write("#pragma once\n\n#include <map>\n#include <string>\n#include \"rigidbody.h\"\n\n")

os.chdir(wrappers_dir)
wrapper_files = glob.glob("*.cpp")

# include all the wrapper files
for file in wrapper_files:
    wrapper_h.write("#include \"../../wrappers/" + file + "\"\n")

# fill out the map
wrapper_h.write("\nstd::map<std::string, unsigned int> droneTypeMap = {\n")
for i in range(len(wrapper_files)):
    wrapper_h.write("    {\"" + wrapper_files[i][:-4] + "\", " + str(i+1) + "}")
    if (i < len(wrapper_files)-1):
        wrapper_h.write(",")
    wrapper_h.write("\n")
wrapper_h.write("};\n\n")

wrapper_h.write("namespace mdp_wrappers {\n")
# write in common function
wrapper_h.write("unsigned int get_drone_type_id(std::string droneTag) {\n")
wrapper_h.write("    std::string droneType = droneTag.substr(0, droneTag.find_first_of('_'));\n    if (droneTypeMap.count(droneType) == 0) {\n        return 0;\n    } else {\n        return droneTypeMap[droneType];\n    }\n}\n\n")

# write in get data descriptions function
wrapper_h.write("std::string get_data_desc(std::string pTag) {\n")
wrapper_h.write("    switch(get_drone_type_id(pTag)) {\n")
for i in range(len(wrapper_files)):
    wrapper_h.write("        case " + str(i+1) + ": return " + wrapper_files[i][:-4] + "::get_data_desc();\n")
wrapper_h.write("        default: return \"\";\n    }\n}\n\n")

# write in the create rigidbody function
wrapper_h.write("bool create_new_rigidbody(std::string pTag, uint32_t id, std::vector<std::string> args, rigidbody* &pRigidbodyPtr) {\n")
wrapper_h.write("    switch(get_drone_type_id(pTag)) {\n")
for i in range(len(wrapper_files)):
    wrapper_h.write("        case " + str(i+1) + ": {pRigidbodyPtr = (rigidbody*)(new " + wrapper_files[i][:-4] + "(pTag, id, args)); return true;}\n")
wrapper_h.write("        default: {pRigidbodyPtr = nullptr; return false;}\n    }\n}\n}\n")

wrapper_h.close()
