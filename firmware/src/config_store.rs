use core::ops::Range;

use defmt::{debug, error};
use embedded_storage::nor_flash::NorFlash;
use pnpfeeder::{ConfigStore, Error, FeederConfig, Value};
use sequential_storage::map::{fetch_item, store_item, StorageItem};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
enum ConfigKey {
    FeederConfigV0(usize),
}

enum ConfigValue {
    FeederConfigV0(FeederConfig),
}

struct ConfigStorageItem {
    key: ConfigKey,
    value: ConfigValue,
}

impl ConfigStorageItem {
    // Key = 2 u32s, FeederConfig =
    const KEY_WORDS: usize = 2;
    const FEEDER_WORDS: usize = 8;
    const PADDING_WORDS: usize = 0;
    const BYTES_PER_WORD: usize = 5;
    const BUFFER_SIZE: usize =
        (Self::KEY_WORDS + Self::FEEDER_WORDS + Self::PADDING_WORDS) * Self::BYTES_PER_WORD;

    fn new_config(index: usize, config: FeederConfig) -> Self {
        Self {
            key: ConfigKey::FeederConfigV0(index),
            value: ConfigValue::FeederConfigV0(config),
        }
    }
}

impl StorageItem for ConfigStorageItem {
    type Key = ConfigKey;
    type Error = Error;

    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        let key_buf = postcard::to_slice(&self.key, buffer).map_err(|_| Error::ConfigSetError)?;
        let key_len = key_buf.len();
        let value_buf = &mut buffer[key_len..];
        let value_buf = match &self.value {
            ConfigValue::FeederConfigV0(config) => {
                postcard::to_slice(&config, value_buf).map_err(|_| Error::ConfigSetError)?
            }
        };

        Ok(key_len + value_buf.len())
    }

    fn deserialize_from(buffer: &[u8]) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        let (key, value_buf) =
            postcard::take_from_bytes(buffer).map_err(|_| Error::ConfigSetError)?;
        let value = match key {
            ConfigKey::FeederConfigV0(_) => {
                let config = postcard::from_bytes(value_buf).map_err(|_| Error::ConfigSetError)?;
                ConfigValue::FeederConfigV0(config)
            }
        };

        Ok(Self { key, value })
    }

    fn key(&self) -> Self::Key {
        self.key.clone()
    }
}

pub struct FlashConfigStore<Flash: NorFlash> {
    flash: Flash,
    range: Range<u32>,
}

impl<Flash: NorFlash> FlashConfigStore<Flash> {
    pub fn new(flash: Flash, range: Range<u32>) -> Self {
        Self { flash, range }
    }

    fn default_config() -> FeederConfig {
        FeederConfig {
            advanced_angle: Value::from_num(135.0),
            half_advanced_angle: Value::from_num(107.5),
            retract_angle: Value::from_num(80),
            feed_length: Value::from_num(2.0),
            settle_time: 300,
            pwm_0: Value::from_num(490.2),
            pwm_180: Value::from_num(980.4),
            ignore_feeback_pin: false,
            always_retract: true,
        }
    }
}

impl<Flash: NorFlash> ConfigStore for FlashConfigStore<Flash> {
    fn get(&mut self, index: usize) -> pnpfeeder::Result<FeederConfig> {
        debug!("config get {}", index);
        let mut buf = [0u8; ConfigStorageItem::BUFFER_SIZE];
        let range = self.range.clone();
        // Too much extraneous error handling here.  We should be able to clean this up.
        let item: Option<ConfigStorageItem> = fetch_item(
            &mut self.flash,
            range,
            &mut buf,
            ConfigKey::FeederConfigV0(index),
        )
        .unwrap_or_else(|e| {
            // On any error, log it and return the default config.
            match e {
                sequential_storage::map::MapError::Item(_) => {
                    error!("config get {} item error", index)
                }
                sequential_storage::map::MapError::Storage(_) => {
                    error!("config get {} storage error", index)
                }
                sequential_storage::map::MapError::FullStorage => {
                    error!("config get {} full storage error", index)
                }
                sequential_storage::map::MapError::Corrupted => {
                    error!("config get {} corrupted error", index)
                }
                sequential_storage::map::MapError::BufferTooBig => {
                    error!("config get {} buffer too big error", index)
                }
                sequential_storage::map::MapError::BufferTooSmall(_) => {
                    error!("config get {} buffer too small error", index)
                }
                _ => error!("config get {} unknown error", index),
            };
            None
        });

        match item
            .map(|item| item.value)
            .unwrap_or(ConfigValue::FeederConfigV0(Self::default_config()))
        {
            ConfigValue::FeederConfigV0(feeder) => Ok(feeder),
        }
    }

    fn set(&mut self, index: usize, config: &FeederConfig) -> pnpfeeder::Result<()> {
        debug!("config set {}", index);
        let mut buf = [0u8; ConfigStorageItem::BUFFER_SIZE];
        let range = self.range.clone();
        let item = ConfigStorageItem::new_config(index, config.clone());
        store_item(&mut self.flash, range, &mut buf, item).map_err(|e| {
            match e {
                sequential_storage::map::MapError::Item(_) => {
                    error!("config get {} item error", index)
                }
                sequential_storage::map::MapError::Storage(_) => {
                    error!("config get {} storage error", index)
                }
                sequential_storage::map::MapError::FullStorage => {
                    error!("config get {} full storage error", index)
                }
                sequential_storage::map::MapError::Corrupted => {
                    error!("config get {} corrupted error", index)
                }
                sequential_storage::map::MapError::BufferTooBig => {
                    error!("config get {} buffer too big error", index)
                }
                sequential_storage::map::MapError::BufferTooSmall(_) => {
                    error!("config get {} buffer too small error", index)
                }
                _ => error!("config get {} unknown error", index),
            };
            Error::ConfigSetError
        })
    }
}
