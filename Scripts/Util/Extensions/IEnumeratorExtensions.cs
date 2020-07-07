using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public static class IEnumeratorExtensions {
    /// <summary>
    /// Execute an entire Unity Coroutine in one frame.
    /// This is useful for testing coroutines with NUnit.
    /// 
    /// The process will bail out after a fixed number of yields to avoid
    /// looping infinitely.
    /// 
    /// Calling StartCoroutine from inside an IEnumerator is not supported
    /// because there is no way to access the IEnumerator object created
    /// by StartCoroutine.
    /// 
    /// Returns true if the coroutine execution finished, otherwise false if it
    /// bailed early after reaching the maximum number of yields.
    /// </summary>
    public static bool RunCoroutineWithoutYields(this IEnumerator enumerator, int maxYields = 1000) {
        Stack<IEnumerator> enumStack = new Stack<IEnumerator>();
        enumStack.Push(enumerator);

        int step = 0;
        while (enumStack.Count > 0) {
            IEnumerator activeEnum = enumStack.Pop();
            while (activeEnum.MoveNext()) {
                if (activeEnum.Current is IEnumerator) {
                    enumStack.Push(activeEnum);
                    activeEnum = (IEnumerator)activeEnum.Current;
                } else if (activeEnum.Current is Coroutine) {
                    throw new System.NotSupportedException("RunCoroutineWithoutYields can not be used with an IEnumerator that calls StartCoroutine inside itself.");
                }
                step += 1;
                if (step >= maxYields) {
                    return false;
                }
            }
        }
        return true;
    }
}
