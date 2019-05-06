import antenna_tracker

antenna_tracker.on_configure_pos({
    'lat': 38.14537361758957,
    'lng': -76.42863630688475,
})

# circle around 3 times
for i in range(3):
    antenna_tracker.track({
        'lat': 38.14614987740943,
        'lng': -76.42192005551146,
        'alt': 100
    })

    antenna_tracker.track({
        'lat': 38.14955857275383,
        'lng': -76.42468809521483,
        'alt': 100
    })

    antenna_tracker.track({
        'lat': 38.14621737787297,
        'lng': -76.43404364025878,
        'alt': 100
    })

    antenna_tracker.track({
        'lat': 38.14140781353844,
        'lng': -76.42816423809813,
        'alt': 100
    })


