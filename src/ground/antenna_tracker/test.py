import antenna_tracker

antenna_tracker.configure_pos({
    'lat': 38.14537361758957,
    'lng': -76.42863630688475,
})

# circle around 3 times
for i in range(3):
    antenna_tracker.track({
        'latitude': 38.14614987740943,
        'longitude': -76.42192005551146,
        'relative_altitude': 100
    })

    print("Next stepper pos should be 329, 329+800=1129, etc.")
    antenna_tracker.track({ # should be in position 180-32=148 degrees * 800/360 steps/degree = 329
        'latitude': 38.158760459481556,
        'longitude': -76.4561290752938,
        'relative_altitude': 100
    })

    antenna_tracker.track({
        'latitude': 38.14621737787297,
        'longitude': -76.43404364025878,
        'relative_altitude': 100
    })

    antenna_tracker.track({
        'latitude': 38.14140781353844,
        'longitude': -76.42816423809813,
        'relative_altitude': 100
    })

    print("Next pitch should be 45 degrees")
    antenna_tracker.track({ # 205.73 meters = 674.95 feet away, 205.73 m altitude -> 45 degrees pitch
        'latitude': 38.14583577798412,
        'longitude': -76.4263825946557,
        'relative_altitude': 205.73
    })
