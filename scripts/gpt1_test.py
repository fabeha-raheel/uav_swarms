import math

# Earth radius in meters
EARTH_RADIUS = 6378137.0  # Approximate value for WGS84 ellipsoid

def calculate_follower_coordinates(leader_latitude, leader_longitude, leader_heading, num_followers):
    follower_coordinates = []

    # Convert latitude and longitude from degrees to radians
    leader_latitude_rad = math.radians(leader_latitude)
    leader_longitude_rad = math.radians(leader_longitude)

    # Convert heading from degrees to radians
    heading_rad = math.radians(leader_heading)

    # Calculate initial change in latitude and longitude (2 meters behind the leader)
    delta_latitude = 2 / EARTH_RADIUS
    delta_longitude = 2 / (EARTH_RADIUS * math.cos(leader_latitude_rad))

    for i in range(num_followers):
        # Calculate new latitude and longitude for each follower
        new_latitude_rad = leader_latitude_rad - i * delta_latitude
        new_longitude_rad = leader_longitude_rad - i * delta_longitude

        # Convert new latitude and longitude from radians to degrees
        new_latitude = math.degrees(new_latitude_rad)
        new_longitude = math.degrees(new_longitude_rad)

        # Add the coordinates to the list
        follower_coordinates.append((new_latitude, new_longitude))

    return follower_coordinates

# Example usage
leader_latitude = 40.7128  # Example latitude of the leader
leader_longitude = -74.0060  # Example longitude of the leader
leader_heading = 45  # Example heading of the leader in degrees (clockwise from true North)
num_followers = 5  # Example number of followers

follower_coordinates = calculate_follower_coordinates(leader_latitude, leader_longitude, leader_heading, num_followers)
print("Follower GPS coordinates:", follower_coordinates)
# for i, (latitude, longitude) in enumerate(follower_coordinates):
#     print(f"Follower {i+1}: {latitude}, {longitude}")
