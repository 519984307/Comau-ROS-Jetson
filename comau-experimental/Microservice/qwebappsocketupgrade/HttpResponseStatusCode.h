#pragma once
#include "qwebappsocketupgrade_global.h"
enum class QWEBAPPSOCKETUPGRADE_EXPORT HttpResponseStatusCode
{
	/**InformationResponses*/
	Continue=100, /** This interim response indicates that the client should continue the request or ignore the response if the request is already finished.*/
	SwitchingProtocols= 101, /** This code is sent in response to an Upgrade request header from the client and indicates the protocol the server is switching to.*/
	Processing=102, /**This code indicates that the server has received and is processing the request, but no response is available yet.*/
	EarlyHints= 103, /**This status code is primarily intended to be used with the Link header, letting the user agent start preloading resources while the server prepares a response. */
	/**Successful responses*/

 /** The request succeeded. The result meaning of "success" depends on the HTTP method:
GET: The resource has been fetched and transmitted in the message body.
HEAD: The representation headers are included in the response without any message body.
PUT or POST: The resource describing the result of the action is transmitted in the message body.
TRACE: The message body contains the request message as received by the server. */
	OK = 200,
	Created= 201, /**The request succeeded, and a new resource created as a result. This is typically the response sent after POST requests, or some PUT requests.*/
	Accepted= 202, /**The request has been received but not yet acted upon. It is noncommittal, since there is no way in HTTP to later send an asynchronous response indicating the outcome of the request. It is intended for cases where another process or server handles the request, or for batch processing.*/
	NonAuthoritativeInformation= 203, /**This response code means the returned metadata is not exactly the same as is available from the origin server, but is collected from a local or a third-party copy. This is mostly used for mirrors or backups of another resource. Except for that specific case, the 200 OK response is preferred to this status.*/
	NoContent= 204,/**There is no content to send for this request, but the headers may be useful. The user agent may update its cached headers for this resource with the new ones*/
	ResetContent=205,/**Tells the user agent to reset the document which sent this request.*/
	PartialContent=206,/**This response code is used when the Range header is sent from the client to request only part of a resource.*/
	/**Redirection messages*/
	MultipleChoices=300, /**The request has more than one possible response. The user agent or user should choose one of them.*/
	MovedPermanently=301,/**The URL of the requested resource has been changed permanently. The new URL is given in the response.*/
	Found= 302,/*This response code means that the URI of requested resource has been changed temporarily. Further changes in the URI might be made in the future. */
	SeeOther= 303,/**The server sent this response to direct the client to get the requested resource at another URI with a GET request.*/
	NotModified= 304,/**This is used for caching purposes. It tells the client that the response has not been modified, so the client can continue to use the same cached version of the response.*/
	TemporaryRedirect= 307,/**The server sends this response to direct the client to get the requested resource at another URI with same method that was used in the prior request. */
	PermanentRedirect= 308,/**This means that the resource is now permanently located at another URI, specified by the Location: HTTP Response header. */

	/**Client error responses*/
	BadRequest= 400, /**The server could not understand the request due to invalid syntax.*/
	Unauthorized= 401, /**the client must authenticate itself to get the requested response*/
	Forbidden=403, /**The client does not have access rights to the content; that is, it is unauthorized, so the server is refusing to give the requested resource.*/
	NotFound= 404, /**The server can not find the requested resource. In the browser, this means the URL is not recognized. In an API, this can also mean that the endpoint is valid but the resource itself does not exist.*/
	MethodNotAllowed= 405,/**The request method is known by the server but is not supported by the target resource. */
	NotAcceptable= 406,/**This response is sent when the web server, after performing server-driven content negotiation, doesn't find any content that conforms to the criteria given by the user agent.*/
	ProxyAuthenticationRequired= 407, /**This is similar to 401 Unauthorized but authentication is needed to be done by a proxy.*/
	RequestTimeout= 408,/**This response is sent on an idle connection by some servers, even without any previous request by the client. */
	Conflict= 409, /**This response is sent when a request conflicts with the current state of the server.*/
	Gone= 410, /**This response is sent when the requested content has been permanently deleted from server, with no forwarding address. */
	LengthRequired= 411, /**Server rejected the request because the Content-Length header field is not defined and the server requires it*/
	PreconditionFailed=412, /**The client has indicated preconditions in its headers which the server does not meet.*/
	PayloadTooLarge=413, /*Request entity is larger than limits defined by server. */
	URITooLong=414, /**The URI requested by the client is longer than the server is willing to interpret.*/
	UnsupportedMediaType=415, /**The media format of the requested data is not supported by the server, so the server is rejecting the request.*/
	RangeNotSatisfiable= 416,/**The range specified by the Range header field in the request cannot be fulfilled*/
	ExpectationFailed= 417,/**This response code means the expectation indicated by the Expect request header field cannot be met by the server.*/
	ImATeapot=418, /**The server refuses the attempt to brew coffee with a teapot.*/
	UnprocessableEntity= 422,/**The request was well-formed but was unable to be followed due to semantic errors.*/
	TooEarly= 425,/**Indicates that the server is unwilling to risk processing a request that might be replayed.*/
	UpgradeRequired= 426,/**The server refuses to perform the request using the current protocol but might be willing to do so after the client upgrades to a different protocol.*/
	PreconditionRequired= 428,/**The origin server requires the request to be conditional. */
	TooManyRequests= 429,/**The user has sent too many requests in a given amount of time*/
	RequestHeaderFieldsTooLarge= 431, /**The server is unwilling to process the request because its header fields are too large. */
	UnavailableForLegalReasons= 451,/**The user agent requested a resource that cannot legally be provided, such as a web page censored by a government*/
	/**Server error responses*/
	InternalServerError= 500, /**The server has encountered a situation it does not know how to handle.*/
	NotImplemented=501,/**The request method is not supported by the server and cannot be handled. */
	BadGateway= 502,/**This error response means that the server, while working as a gateway to get a response needed to handle the request, got an invalid response*/
	ServiceUnavailable= 503,/**The server is not ready to handle the request*/
	GatewayTimeout= 504,/**This error response is given when the server is acting as a gateway and cannot get a response in time.*/
	HTTPVersionNotSupported=505,/**The HTTP version used in the request is not supported by the server*/
	VariantAlsoNegotiates= 506,/**The server has an internal configuration error: the chosen variant resource is configured to engage in transparent content negotiation itself, and is therefore not a proper end point in the negotiation process.*/
	InsufficientStorage= 507,/**The method could not be performed on the resource because the server is unable to store the representation needed to successfully complete the request*/
	LoopDetected= 508,/**The server detected an infinite loop while processing the request.*/
	NotExtended= 510,/**Further extensions to the request are required for the server to fulfill it.*/
	NetworkAuthenticationRequired= 511,/*Indicates that the client needs to authenticate to gain network access.*/

};

