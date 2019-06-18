import ravestate as rs
import ravestate_nlp as nlp


prop_flavor = None
prop_scoops = None


@rs.state(
    cond=nlp.prop_tokens.changed(),
    read=nlp.prop_tokens)
def detect_flavor(ctx: rs.ContextWrapper):
    tokens = ctx[nlp.prop_tokens]
    if "chocolate" in tokens or "vanilla" in tokens:
        # TODO fill property flavor
        pass


@rs.state(
    cond=nlp.prop_tokens.changed(),
    read=nlp.prop_tokens)
def detect_scoops(ctx: rs.ContextWrapper):
    # TODO fill property scoops, maybe change cond to react on other things?
    pass


# sig_flavor_detected = rs.Signal(name="flavor detected")
# sig_payment_detected = rs.Signal(name="payment detected")
# sig_predicate_want = rs.Signal(name="predicate want")
#
#
# @rs.state(
#     cond=nlp.prop_tokens.changed(),
#     read=nlp.prop_tokens,
#     signal=sig_flavor_detected)
# def flavor_signal(ctx: rs.ContextWrapper):
#     tokens = ctx[nlp.prop_tokens]
#     if "chocolate" in tokens or "vanilla" in tokens:
#         return rs.Emit()
#
#
# @rs.state(
#     cond=nlp.prop_tokens.changed(),
#     read=nlp.prop_tokens,
#     signal=sig_payment_detected)
# def payment_signal(ctx: rs.ContextWrapper):
#     tokens = ctx[nlp.prop_tokens]
#     if "bitcoin" in tokens or "credit" in tokens or "cash" in tokens:
#         return rs.Emit()
#
#
# @rs.state(
#     cond=nlp.prop_triples.changed(),
#     read=nlp.prop_triples,
#     signal=sig_predicate_want)
# def predicate_want_signal(ctx: rs.ContextWrapper):
#     # TODO include questions like "Can I pay with cash, please" or "Can I have chocolate, please"
#     triples = ctx[nlp.prop_triples]
#     if triples[0].match_either_lemma(pred={"want", "like", "desire", "have", "pay"}) and \
#             triples[0].match_either_lemma(subj={"i"}):
#         return rs.Emit()
#
#
# @rs.state(
#     cond=sig_flavor_detected & sig_predicate_want,
#     write=rawio.prop_out)
# def flavor_order_received(ctx: rs.ContextWrapper):
#     ctx[rawio.prop_out] = "Flavor Order"
#
#
# @rs.state(
#     cond=sig_payment_detected & sig_predicate_want,
#     write=rawio.prop_out)
# def payment_request_received(ctx: rs.ContextWrapper):
#     ctx[rawio.prop_out] = "Payment Request"
#
#
# @rs.state(
#     cond=sig_flavor_detected & nlp.sig_is_question,
#     write=rawio.prop_out)
# def flavor_question_received(ctx: rs.ContextWrapper):
#     # TODO extend to understand questions like "Do you have chocolate ice cream?"
#     # TODO nlp.sig_is_question uses only question words like "what" or "where" etc. Maybe add others?
#     ctx[rawio.prop_out] = "Flavor Question"
#
#
# @rs.state(
#     cond=sig_payment_detected & nlp.sig_is_question,
#     write=rawio.prop_out)
# def payment_question_received(ctx: rs.ContextWrapper):
#     ctx[rawio.prop_out] = "Payment Question"
