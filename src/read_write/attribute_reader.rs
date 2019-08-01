use crate::proto;
use crate::AttributeData;
use std::io::{Read, BufReader};

pub fn attr_from_proto<R : Read>(proto : proto::Attribute, attr_source: &R)-> std::io::Result<(String, AttributeData)> {
    let attr_data = match proto.data_type {
       proto::AttributeDataType::I64 => AttributeData::I64(read_source(attr_source)?),
       proto::AttributeDataType::U64 => AttributeData::U64(read_source(attr_source)?),
       proto::AttributeDataType::F32 => AttributeData::F32(read_source(attr_source)?),
       proto::AttributeDataType::F64 => AttributeData::F64(read_source(attr_source)?),
       proto::AttributeDataType::U8Vec3 => AttributeData::U8Vec3(read_source(attr_source)?),
       proto::AttributeDataType::F64Vec3 => AttributeData::F64Vec3(read_source(attr_source)?),
    };
    Ok((proto.name, attr_data))
}

fn read_source<R : Read, T>(attr_source: R) -> std::io::Result<Vec<T>>{
   let mut vec = Vec::<T>::new();
   let bufreader = BufReader::new(attr_source);
   bufreader.read_to_end(&mut vec)?;
   Ok(vec)
}
