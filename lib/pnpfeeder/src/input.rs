pub trait Input {
    #[allow(async_fn_in_trait)]
    async fn wait_for_high(&mut self);

    #[allow(async_fn_in_trait)]
    async fn wait_for_low(&mut self);

    #[allow(async_fn_in_trait)]
    async fn wait_for_state_change(&mut self);

    #[allow(async_fn_in_trait)]
    async fn get_state(&mut self) -> bool;
}
