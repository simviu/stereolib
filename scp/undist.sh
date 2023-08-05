
CAML=oakdl_left.yml
CAMR=oakdl_right.yml

./vsntool image undist dir=rund/frms/left_ori dirw=rund/frms/left_ud cfg=rund/cam/$CAML
./vsntool image undist dir=rund/frms/right_ori dirw=rund/frms/right_ud cfg=rund/cam/$CAML

