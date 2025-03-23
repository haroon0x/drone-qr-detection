def get_distance_metres(Location1, Location2):
    """
    Returns the ground distance(horizontal) in meters between two LocationGlobalRelative points.
    -----------
    chat-gpt - generated func
    -----------
    """
    from math import radians, sin, cos, sqrt, atan2

    R = 6371000  # Radius of Earth in meters

    # Convert latitude and longitude to radians
    lat1, lon1 = radians(Location1.lat), radians(Location1.lon)
    lat2, lon2 = radians(Location2.lat), radians(Location2.lon)
    
    # Compute differences
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Haversine formula to calculate distance
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    return R * c  # Returns distance in meters