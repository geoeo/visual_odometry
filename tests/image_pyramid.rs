use visual_odometry::image_pyramid::Layer;

#[test]
fn depth_coordiantes(){
    let width = 640;
    let height = 480;
    let width_mid_x = width/2;
    let width_mid_y = height/2;

    let width_small = width/4;
    let height_small = height/4;
    let width_mid = width/8;
    let height_mid = height/8;

    let mid_x_small = width_small/2;
    let mid_y_small = height_small/2;
    let mid_x_mid = width_mid/2;
    let mid_y_mid = height_mid/2;

    let x_sample = 15;
    let y_sample = 35;

    let (x_high, y_high) = Layer::generate_depth_coordiantes(2,mid_x_small,mid_y_small);
    let (x_high_2, y_high_2) = Layer::generate_depth_coordiantes(3,mid_x_mid,mid_y_mid);
    //let (x_high_3, y_high_3) = Layer::generate_depth_coordiantes(3,x_sample,y_sample);

    assert_eq!(x_high,width_mid_x);
    assert_eq!(y_high,width_mid_y);
    assert_eq!(x_high_2,width_mid_x);
    assert_eq!(y_high_2,width_mid_y);
}