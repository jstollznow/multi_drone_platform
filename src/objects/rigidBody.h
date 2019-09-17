#include <string>

class rigidBody
{
    private:
        int global_id = 0;
        // platformID
        int platform_id;
        //rigid body tag
        std::string optitrackTag;
        float* velocity;
    public: 
        rigidBody(std::string tag);
        ~rigidBody();
    // std::vector<rigidBody*> arr;
    // rigidBody* body = arr[0];
    // if (body->isDrone()) {
    //     drone* d = (drone*)body;
    // }
};