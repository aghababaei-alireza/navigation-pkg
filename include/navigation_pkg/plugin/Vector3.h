#define NAVIGATION_PKG_MESSAGE_VECTOR3_PLUGIN_CONSTRUCTOR \
    Vector3_(double _x=0, double _y=0, double _z=0) \
    :x(_x), y(_y), z(_z) {} 

#define NAVIGATION_PKG_MESSAGE_VECTOR3_PLUGIN_CLASS_BODY \
    void Set(double _x, double _y, double _z) \
    { \
        x = _x; \
        y = _y; \
        z = _z; \
    } \
    \
    void Reset() \
    { \
        x = 0.0; \
        y = 0.0; \
        z = 0.0; \
    }
