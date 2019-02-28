type Task<T> = {
  fn: Function,
  resolve: Function,
  reject: Function
};

export default class AsyncTasksQueue<T> {

  private queue: Task<T>[];
  private pendingCount: number;

  constructor(
    public maxConcurrentTasks: number
  ) {
    this.queue = [];
    this.pendingCount = 0;
  }

  public async addTask(fn: () => Promise<T>): Promise<T> {
    return new Promise<T>((resolve, reject) => {
      this.queue.push({ fn, resolve, reject });
      this.processQueue();
    });
  }

  private processQueue() {
    this.queue.splice(0, this.maxConcurrentTasks - this.pendingCount)
      .forEach(
        ({ fn, resolve, reject }) => {
          this.pendingCount++;
          fn()
            .then(resolve, reject)
            .finally(() => {
              this.pendingCount--;
              this.processQueue();
            });
        }
      );
  }
}


