komodo151
=========

Algo blocks:
```
while ( ! cup ) {
  Recognize:
    take picture
    match
    if ( cup ) go_get_cup
  Rotate
    // actual rotation angle depends on camera angle
  if ( full rotation completed )
    Random Walk
}
```

`go_get_cup` repeats until having the rover bump into something:
1. fix rover rotation based on cup location in the image, and cup distance
2. drive one distance unit

* Drive distance can be a function of the distance to the cup (e.g. 1/2 dist)
* can validate rotation amount by another cup recognition.
