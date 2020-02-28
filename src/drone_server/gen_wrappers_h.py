import glob, os
print("Generating drone wrapper header file")
wrappers_dir_rel = "../wrappers"
wrappers_dir = os.path.dirname(os.path.realpath(__file__)) + "/" + wrappers_dir_rel

wrapper_h = open(os.path.dirname(os.path.realpath(__file__)) + "/wrappers.h", 'wt')
wrapper_h.write("#pragma once\n\n#include <map>\n#include <string>\n#include \"../objects/rigidBody.h\"\n\n")

os.chdir(wrappers_dir)
wrapper_files = glob.glob("*.cpp")

# include all the wrapper files
for file in wrapper_files:
    wrapper_h.write("#include \"" + wrappers_dir_rel + "/" + file + "\"\n")

# fill out the map
wrapper_h.write("\nstd::map<std::string, unsigned int> drone_type_map = {\n")
for i in range(len(wrapper_files)):
    wrapper_h.write("    {\"" + wrapper_files[i][:-4] + "\", " + str(i+1) + "}")
    if (i < len(wrapper_files)-1):
        wrapper_h.write(",")
    wrapper_h.write("\n")
wrapper_h.write("};\n\n")

# write in the create rigidbody function
wrapper_h.write("namespace mdp_wrappers{\nbool createNewRigidbody(std::string pTag, uint32_t id, rigidBody* &pRigidbodyPtr)\n{\n")
wrapper_h.write("    std::string DroneType = pTag.substr(0, pTag.find_first_of('_'));\n    switch(drone_type_map[DroneType]) {\n")
for i in range(len(wrapper_files)):
    wrapper_h.write("        case " + str(i+1) + ": {pRigidbodyPtr = (rigidBody*)(new " + wrapper_files[i][:-4] + "(pTag, id)); return true;}\n")
wrapper_h.write("        default: {pRigidbodyPtr = nullptr; return false;}\n    }\n}\n}\n")

wrapper_h.close()
