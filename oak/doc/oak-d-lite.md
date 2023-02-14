## ROS ref
https://qiita.com/nindanaoto/items/20858eca08aad90b5bab

### Calc Camera.bf (qiita)
b = -7.47539711 * 0.01 , b_spec = 0.075
fx2 = 843.39074707 (resized intrinsic)
bf2 = 63.046807532

fx = 836.39874268 (resized intrinsic)
fx*b = 62.524127438
fx*b_spec = 62.729905701

expect Camera.bf: 62.729905701 ( fx*b_spec )

## Calc Camera.bf (Oak-D-Lite)
b_spec = 0.075
fx = 910.97033691  (resized intrinsic)
fx * b_spec = 68.322775268


