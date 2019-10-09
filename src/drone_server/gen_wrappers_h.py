import glob, os

wrappers_dir_rel = "../wrappers"
wrappers_dir = os.path.dirname(os.path.realpath(__file__)) + "/" + wrappers_dir_rel

wrapper_h = open(os.path.dirname(os.path.realpath(__file__)) + "/wrappers.h", 'wt')
wrapper_h.write("#pragma once\n\n#include <map>\n#include \"../objects/rigidBody.h\"\n\n")

os.chdir(wrappers_dir)
wrapper_files = glob.glob("*.cpp")

# include all the wrapper files
for file in wrapper_files:
    wrapper_h.write("#include \"" + wrappers_dir_rel + "/" + file + "\"\n")

# fill out the map
wrapper_h.write("\nstd::map<const char*, unsigned int> drone_type_map = {\n")
for i in range(len(wrapper_files)):
    wrapper_h.write("    {\"" + wrapper_files[i][:-4] + "\", " + str(i) + "}")
    if (i < len(wrapper_files)-1):
        wrapper_h.write(",")
    wrapper_h.write("\n")
wrapper_h.write("};\n\n")

# write in the create rigidbody function
wrapper_h.write("namespace mdp_wrappers{\nrigidBody* createNewRigidbody(std::string pTag)\n{\n")
wrapper_h.write("    std::string DroneType = pTag.substr(0, pTag.find_last_of('_'));\n    switch(drone_type_map[DroneType.c_str()]) {\n")
for i in range(len(wrapper_files)):
    wrapper_h.write("        case " + str(i) + ": {return (rigidBody*)(new " + wrapper_files[i][:-4] + "(pTag));}\n")
wrapper_h.write("        default: return nullptr;\n    }\n}\n}\n")

wrapper_h.close()
