using System;

public class ActionClientException : Exception
{
    public ActionClientException() : base()
    {
    }

    public ActionClientException(string message) : base(message)
    {
    }
}
