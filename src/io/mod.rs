extern crate nalgebra as na;

use na::Vector6;
use crate::Float;
use byteorder::{BigEndian, ReadBytesExt};
use png::{self, HasParameters};
use std::{self,path::{Path,PathBuf}};
use std::io::{self,Cursor,prelude::*};
use std::fs::{File,OpenOptions,read_dir};

// https://github.com/mpizenberg/visual-odometry-rs/blob/master/src/misc/helper.rs#L13
/// Read a 16 bit gray png image from a file.
pub fn read_png_16bits_row_major<P: AsRef<Path>>(
    file_path: P,
) -> Result<(usize, usize, Box<Vec<u16>>), png::DecodingError> {
    // Load 16 bits PNG depth image.
    let img_file = File::open(file_path)?;
    let mut decoder = png::Decoder::new(img_file);
    // Use the IDENTITY transformation because by default
    // it will use STRIP_16 which only keep 8 bits.
    // See also SWAP_ENDIAN that might be useful
    //   (but seems not possible to use according to documentation).
    decoder.set(png::Transformations::IDENTITY);
    let (info, mut reader) = decoder.read_info()?;
    let mut buffer = vec![0; info.buffer_size()];
    reader.next_frame(&mut buffer)?;

    // Transform buffer into 16 bits slice.
    // Transform buffer into 16 bits slice.
    // if cfg!(target_endian = "big") ...
    let mut buffer_u16 = vec![0; (info.width * info.height) as usize];
    let mut buffer_cursor = Cursor::new(buffer);
    buffer_cursor.read_u16_into::<BigEndian>(&mut buffer_u16)?;

    // Return u16 buffer.
    Ok((info.width as usize, info.height as usize, Box::new(buffer_u16)))
}


pub fn get_file_list_in_dir<P: AsRef<Path>>(image_folder_path: P) -> io::Result<Vec<String>> {

    let mut file_vec: Vec<String> = Vec::new();
    for entry in read_dir(image_folder_path)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir(){
            continue;
        } else {
            let name = entry.file_name().into_string().unwrap_or_else(|_| panic!("Error reading filename"));
            if is_valid_file(&name) {
                file_vec.push(name);
            }
        }
    }

    Ok(file_vec)
}

pub fn file_name_to_float(filename: &str) -> f64 {
    let splits = filename.split(".");
    let vec: Vec<&str> = splits.collect();
    let float_str = format!("{}.{}",vec[0],vec[1]);
    float_str.parse().unwrap_or_else(|_| panic!("unable to convert filename to float"))
}

pub fn associate_file_name<P: AsRef<Path>>(image_folder_path: P, time_stamp: f64, max_diff_milliseconds: f64)-> String {
    let file_name_list = get_file_list_in_dir(image_folder_path).unwrap_or_else(|_|panic!("file association failed with: {}",time_stamp));
    let time_stamp_differences: Vec<f64>
        = file_name_list.iter()
        .map(|x| file_name_to_float(x))
        .map(|x| (time_stamp-x).abs())
        .collect();

    let (closet_match_idx, ts_diff_min)
        = time_stamp_differences
        .iter()
        .enumerate()
        .fold((0, 100.0), |(ts_idx, acc), (i, x)| if *x < acc { (i, *x) } else { (ts_idx, acc) });

    if ts_diff_min >= max_diff_milliseconds {
        panic!(
            "The timestamp difference between intensity and depth image is too large for intensity: {}. The difference is: {}, with max diff: {}",
            time_stamp,ts_diff_min,max_diff_milliseconds);
    }

    file_name_list[closet_match_idx].clone()
}

pub fn generate_folder_path(root: PathBuf, folder_path_relative_to_project: &str) -> PathBuf {
    let mut image_folder_path = root;
    image_folder_path.push(folder_path_relative_to_project);
    image_folder_path
}

pub fn generate_runtime_intensity_depth_lists<P: AsRef<Path>>(intensity_folder_path: P, depth_folder_path: P, start_file_name: &str, extension: &str,step_count: usize, frame_count: usize, max_diff_milliseconds: f64)
                                              -> (Vec<String>, Vec<String>) {

    let start_file_as_float: f64 = start_file_name.parse().unwrap_or_else(|_| panic!("unable to convert filename to float"));
    let color_files = get_file_list_in_dir(&intensity_folder_path).unwrap_or_else(|_|panic!("reading files failed"));
    let mut color_files_as_floats: Vec<f64> = color_files.iter().map(|x| file_name_to_float(x)).collect();
    color_files_as_floats.sort_by(|&a,b| a.partial_cmp(b).unwrap());


    let (start_idx,_) = color_files_as_floats.iter().enumerate().find(|(_,x)| **x==start_file_as_float).unwrap_or_else(||panic!("reading files failed"));
    assert!(start_idx+frame_count <= color_files_as_floats.len());

    let mut selected_color_files: Vec<f64> = Vec::new();
    for i in (start_idx..(start_idx+frame_count)).step_by(step_count) {
        selected_color_files.push(color_files_as_floats[i].clone());
    }
    (selected_color_files.iter().map(|&x| float_to_string(x)).map(|mut x| {x.push('.');x.push_str(extension);return x}).collect(),
     selected_color_files.iter().map(|&x| associate_file_name(&depth_folder_path,x,max_diff_milliseconds)).collect())
}

pub fn generate_runtime_paths(intensity_folder_path: PathBuf,
                              depth_folder_path: PathBuf,
                              reference_intensity_files: Vec<String>,
                              reference_depth_files: Vec<String>,
                              target_intensity_files :Vec<String>,
                              target_depth_files: Vec<String>)
    -> (Vec<PathBuf>,Vec<PathBuf>,Vec<PathBuf>,Vec<PathBuf>) {

    let reference_intensity_paths
        = reference_intensity_files.iter().map(|file_name| {
        let mut full_path = intensity_folder_path.clone();
        full_path.push(file_name);
        return full_path
    }).collect();
    let reference_depth_paths
        = reference_depth_files.iter().map(|file_name| {
        let mut full_path = depth_folder_path.clone();
        full_path.push(file_name);
        return full_path
    }).collect();

    let target_intensity_paths
        = target_intensity_files.iter().map(|file_name| {
        let mut full_path = intensity_folder_path.clone();
        full_path.push(file_name);
        return full_path
    }).collect();
    let target_depth_paths
        = target_depth_files.iter().map(|file_name| {
        let mut full_path = depth_folder_path.clone();
        full_path.push(file_name);
        return full_path
    }).collect();

    (reference_intensity_paths,reference_depth_paths,target_intensity_paths,target_depth_paths)

}

//TODO: investigate Cursor trait
pub fn write_lie_vectors_to_file(file_path: &str, data_vec: Vec<Vector6<Float>>) -> () {
    let mut file = OpenOptions::new()
        .create(true)
        .write(true)
        .open(file_path)
        .unwrap();
    for data in data_vec {
        let mut data_as_string = String::new();
        data_as_string.push_str(&(*data.index(0).to_string()));
        data_as_string.push(',');
        data_as_string.push_str(&(*data.index(1).to_string()));
        data_as_string.push(',');
        data_as_string.push_str(&(*data.index(2).to_string()));
        data_as_string.push(',');
        data_as_string.push_str(&(*data.index(3).to_string()));
        data_as_string.push(',');
        data_as_string.push_str(&(*data.index(4).to_string()));
        data_as_string.push(',');
        data_as_string.push_str(&(*data.index(5).to_string()));
        data_as_string.push('\n');
        file.write(data_as_string.as_bytes()).expect("Failed to write lie data to file");
    }
}

// Some times MacOS will prefix files on non APFS file systems with "._"
fn has_win_prefix(file_name: &str) -> bool {
    let prefix = &file_name[0..2];
    prefix == "._"
}

fn is_valid_file(file_name: &str) -> bool {
    !(file_name == ".DS_Store" || has_win_prefix(file_name))
}

// Currently 6 decimal places is hard coded
fn float_to_string(x: f64) -> String {
    format!("{:.6}", x)
}

