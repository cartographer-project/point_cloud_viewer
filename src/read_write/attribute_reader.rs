use crate::proto;
use crate::AttributeData;
use::io::{BufReader, Read, Result};

pub fn attr_from_proto(proto : proto::Attribute, attr_source: Box<dyn Read>)-> io::Result<(String, AttributeData)> {
    let attr_type = match proto.attr_type {
       proto::AttributeData::I64 => AttributeData::I64(read_source(attr_source)?),
       proto::AttributeData::U64 => AttributeData::U64(read_source(attr_source)?),
       proto::AttributeData::F32 => AttributeData::F32(read_source(attr_source)?),
       proto::AttributeData::F64 => AttributeData::F64(read_source(attr_source)?),
       proto::AttributeData::U8Vec3 => AttributeData::U8Vec3(read_source(attr_source)?),
       proto::AttributeData::F64Vec3 => AttributeData::F64Vec3(read_source(attr_source)?),
    };
    Ok((proto.attr_name, attr_type))
}

fn read_source<T>(attr_source: Box<dyn Read>) -> io::Result<Vec<T>>{
   let bufreader = BufReader::<T>::new(attr_source);
   let mut vec = Vec::<T>::new();
   bufreader.read_all(&mut vec)?;
   vec
}