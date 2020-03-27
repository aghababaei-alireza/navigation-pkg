#define NAVIGATION_PKG_MESSAGE_VECTOR2_PLUGIN_CONSTRUCTOR \
    Vector2_(double _x = 0, double _y = 0) \
    :x(_x), y(_y) {} 

#define NAVIGATION_PKG_MESSAGE_VECTOR2_PLUGIN_CLASS_BODY \
    void Set(double _x, double _y) \
    { \
        x = _x; \
        y = _y; \
    } \
    \
    void Reset() \
    { \
        x = 0.0; \
        y = 0.0; \
    }
