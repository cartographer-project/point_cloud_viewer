/**
 * Chunks an array into several arrays of a given size.
 */
export function chunk<T>(arr: T[], size: number): T[][] {
  const result = [];

  if (size >= 1) {
    for (let i = 0; i < arr.length; ) {
      result.push(arr.slice(i, (i += size)));
    }
  }

  return result;
}
